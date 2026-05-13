#include "pick/viewer_payload_builder.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "catcheye/input/pixel_format.hpp"

namespace catcheye::pick {
namespace {

constexpr int WEBSOCKET_CAMERA_MAX_WIDTH = 1280;
constexpr int WEBSOCKET_CAMERA_MAX_HEIGHT = 720;

cv::Mat frame_to_bgr(const catcheye::input::Frame& frame)
{
    if (frame.empty() || frame.width <= 0 || frame.height <= 0 || frame.stride <= 0) {
        return {};
    }

    const std::size_t expected_size = catcheye::input::frame_data_size(frame.format, frame.stride, frame.height);
    if (frame.data.size() < expected_size) {
        return {};
    }

    auto* raw = const_cast<std::uint8_t*>(frame.data.data());
    switch (frame.format) {
    case catcheye::input::PixelFormat::BGR: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        return wrapped.clone();
    }
    case catcheye::input::PixelFormat::RGB: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::RGBA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::BGRA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::GRAY8: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::NV12: {
        cv::Mat wrapped(frame.height + (frame.height / 2), frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
    }
    case catcheye::input::PixelFormat::UNKNOWN:
        break;
    }
    return {};
}

std::vector<std::uint8_t> encode_jpeg(const cv::Mat& bgr)
{
    if (bgr.empty()) {
        throw std::runtime_error("cannot encode empty frame");
    }
    std::vector<std::uint8_t> jpeg;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", bgr, jpeg, params)) {
        throw std::runtime_error("failed to encode JPEG frame");
    }
    return jpeg;
}

cv::Mat limit_camera_payload_resolution(const cv::Mat& bgr)
{
    if (bgr.cols <= WEBSOCKET_CAMERA_MAX_WIDTH && bgr.rows <= WEBSOCKET_CAMERA_MAX_HEIGHT) {
        return bgr;
    }

    const double scale = std::min(
        static_cast<double>(WEBSOCKET_CAMERA_MAX_WIDTH) / static_cast<double>(bgr.cols),
        static_cast<double>(WEBSOCKET_CAMERA_MAX_HEIGHT) / static_cast<double>(bgr.rows));
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    return resized;
}

cv::Mat normalize_u16_frame_to_bgr(const meere::sensor::sptr_frame& frame)
{
    const auto basic = meere::sensor::frame_cast_basic16u(frame);
    if (!basic || !basic->frameData() || basic->frameData()->empty()) {
        return {};
    }

    const int width = basic->frameWidth();
    const int height = basic->frameHeight();
    const auto* data = basic->frameData()->data();
    cv::Mat raw(height, width, CV_16UC1, const_cast<meere::sensor::int16u*>(data));
    cv::Mat normalized;
    cv::normalize(raw, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat colored;
    cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
    return colored;
}

cv::Mat rgb_frame_to_bgr(const meere::sensor::sptr_frame& frame)
{
    const auto basic = meere::sensor::frame_cast_basic8u(frame);
    if (!basic || !basic->frameData() || basic->frameData()->empty() || frame->frameFormat() != "RGB888") {
        return {};
    }

    const int width = basic->frameWidth();
    const int height = basic->frameHeight();
    const auto* data = basic->frameData()->data();
    cv::Mat rgb(height, width, CV_8UC3, const_cast<meere::sensor::int8u*>(data));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}

void write_float32_le(std::uint8_t*& output, float value)
{
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(output, &value, sizeof(value));
        output += sizeof(value);
        return;
    }

    const auto bits = std::bit_cast<std::uint32_t>(value);
    output[0] = static_cast<std::uint8_t>(bits & 0xFFU);
    output[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
    output[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
    output[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
    output += sizeof(value);
}

ViewerPayload cubeeye_pointcloud_payload(const CubeEyeFrameEntry& entry, int downsample)
{
    if (downsample <= 0) {
        throw std::runtime_error("pointcloud downsample must be positive");
    }

    const auto pointcloud = meere::sensor::frame_cast_pcl32f(entry.frame);
    if (!pointcloud || !pointcloud->frameDataX() || !pointcloud->frameDataY() || !pointcloud->frameDataZ()) {
        throw std::runtime_error("CubeEye PointCloud frame is not F32 XYZ");
    }

    const int width = pointcloud->frameWidth();
    const int height = pointcloud->frameHeight();
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("CubeEye PointCloud frame has invalid dimensions");
    }

    const auto* xs = pointcloud->frameDataX();
    const auto* ys = pointcloud->frameDataY();
    const auto* zs = pointcloud->frameDataZ();
    const auto expected_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (xs->size() < expected_count || ys->size() < expected_count || zs->size() < expected_count) {
        throw std::runtime_error("CubeEye PointCloud XYZ arrays are smaller than frame dimensions");
    }

    const auto sampled_width = (width + downsample - 1) / downsample;
    const auto sampled_height = (height + downsample - 1) / downsample;
    const auto point_count = static_cast<std::uint64_t>(sampled_width) * static_cast<std::uint64_t>(sampled_height);
    std::vector<std::uint8_t> bytes;
    bytes.resize(static_cast<std::size_t>(point_count) * 3U * sizeof(float));
    std::uint8_t* output = bytes.data();
    const auto* x_data = xs->data();
    const auto* y_data = ys->data();
    const auto* z_data = zs->data();

    for (int y = 0; y < height; y += downsample) {
        for (int x = 0; x < width; x += downsample) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            write_float32_le(output, x_data[index]);
            write_float32_le(output, y_data[index]);
            write_float32_le(output, z_data[index]);
        }
    }

    return ViewerPayload{
        .name = entry.spec.name,
        .kind = cubeeye_frame_label(entry.spec.type),
        .encoding = "pointcloud_xyz_f32",
        .width = width,
        .height = height,
        .point_count = point_count,
        .stride = downsample,
        .source_timestamp_ms = static_cast<std::uint64_t>(entry.frame->timestamp()),
        .bytes = std::move(bytes),
    };
}

} // namespace

ViewerPayload camera_payload(const catcheye::input::Frame& frame)
{
    const cv::Mat bgr = limit_camera_payload_resolution(frame_to_bgr(frame));
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert Camera Module 3 frame");
    }
    return ViewerPayload{
        .name = "camera_module_3",
        .kind = "camera",
        .encoding = "jpeg",
        .width = bgr.cols,
        .height = bgr.rows,
        .point_count = 0,
        .stride = 1,
        .source_timestamp_ms = static_cast<std::uint64_t>(frame.timestamp),
        .bytes = encode_jpeg(bgr),
    };
}

ViewerPayload cubeeye_payload(const CubeEyeFrameEntry& entry, int pointcloud_downsample)
{
    if (entry.spec.type == meere::sensor::FrameType::PointCloud) {
        return cubeeye_pointcloud_payload(entry, pointcloud_downsample);
    }

    cv::Mat bgr;
    if (entry.spec.type == meere::sensor::FrameType::RGB) {
        if (entry.frame->frameDataType() != meere::sensor::DataType::U8) {
            throw std::runtime_error("CubeEye RGB frame is not U8");
        }
        bgr = rgb_frame_to_bgr(entry.frame);
    } else {
        if (entry.frame->frameDataType() != meere::sensor::DataType::U16) {
            throw std::runtime_error("CubeEye " + cubeeye_frame_label(entry.spec.type) + " frame is not U16");
        }
        bgr = normalize_u16_frame_to_bgr(entry.frame);
    }
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert CubeEye " + cubeeye_frame_label(entry.spec.type) + " frame");
    }

    return ViewerPayload{
        .name = entry.spec.name,
        .kind = cubeeye_frame_label(entry.spec.type),
        .encoding = "jpeg",
        .width = bgr.cols,
        .height = bgr.rows,
        .point_count = 0,
        .stride = 1,
        .source_timestamp_ms = static_cast<std::uint64_t>(entry.frame->timestamp()),
        .bytes = encode_jpeg(bgr),
    };
}

} // namespace catcheye::pick
