#include "pick/viewer_payload_builder.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "catcheye/input/pixel_format.hpp"

namespace catcheye::pick {
namespace {

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

cv::Matx33f rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg)
{
    constexpr float deg_to_rad = static_cast<float>(CV_PI) / 180.0F;
    const float roll = roll_deg * deg_to_rad;
    const float pitch = pitch_deg * deg_to_rad;
    const float yaw = yaw_deg * deg_to_rad;
    const float sr = std::sin(roll);
    const float cr = std::cos(roll);
    const float sp = std::sin(pitch);
    const float cp = std::cos(pitch);
    const float sy = std::sin(yaw);
    const float cy = std::cos(yaw);
    const cv::Matx33f rx(1.0F, 0.0F, 0.0F, 0.0F, cr, -sr, 0.0F, sr, cr);
    const cv::Matx33f ry(cp, 0.0F, sp, 0.0F, 1.0F, 0.0F, -sp, 0.0F, cp);
    const cv::Matx33f rz(cy, -sy, 0.0F, sy, cy, 0.0F, 0.0F, 0.0F, 1.0F);
    return rz * ry * rx;
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

bool point_in_viewer_roi(float x, float y, float z, const PointCloudRoiConfig& roi)
{
    if (!roi.enabled || !roi.apply_to_viewer) {
        return true;
    }
    return x >= roi.min_x_m && x <= roi.max_x_m && y >= roi.min_y_m && y <= roi.max_y_m && z >= roi.min_z_m && z <= roi.max_z_m;
}

ViewerPayload cubeeye_pointcloud_payload(const CubeEyeFrameEntry& entry, int downsample, const PointCloudRoiConfig& pointcloud_roi)
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

    std::vector<std::uint8_t> bytes;
    bytes.reserve(((expected_count + static_cast<std::size_t>(downsample) - 1U) / static_cast<std::size_t>(downsample)) * 3U * sizeof(float));
    const auto* x_data = xs->data();
    const auto* y_data = ys->data();
    const auto* z_data = zs->data();
    std::uint64_t point_count = 0;

    for (int y = 0; y < height; y += downsample) {
        for (int x = 0; x < width; x += downsample) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            if (!point_in_viewer_roi(x_data[index], y_data[index], z_data[index], pointcloud_roi)) {
                continue;
            }
            const std::size_t offset = bytes.size();
            bytes.resize(offset + (3U * sizeof(float)));
            std::uint8_t* output = bytes.data() + offset;
            write_float32_le(output, x_data[index]);
            write_float32_le(output, y_data[index]);
            write_float32_le(output, z_data[index]);
            ++point_count;
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

ViewerPayload camera_payload(const catcheye::input::Frame& frame, const RgbIntrinsicConfig& rgb_intrinsic)
{
    cv::Mat bgr = frame_to_bgr(frame);
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert Camera Module 3 frame");
    }
    if (rgb_intrinsic.undistort_enabled) {
        const cv::Mat camera_matrix =
            (cv::Mat_<double>(3, 3) << rgb_intrinsic.fx, 0.0, rgb_intrinsic.cx, 0.0, rgb_intrinsic.fy, rgb_intrinsic.cy, 0.0, 0.0, 1.0);
        const cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << rgb_intrinsic.dist_k1, rgb_intrinsic.dist_k2, rgb_intrinsic.dist_p1,
                                     rgb_intrinsic.dist_p2, rgb_intrinsic.dist_k3);
        cv::Mat undistorted;
        cv::undistort(bgr, undistorted, camera_matrix, dist_coeffs, camera_matrix);
        bgr = std::move(undistorted);
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

std::optional<ViewerPayload> projected_depth_payload(
    const catcheye::input::Frame& camera_frame,
    const CubeEyeFrameEntry& depth_entry,
    const std::optional<CubeEyeIntrinsics>& cubeeye_intrinsics,
    const RgbIntrinsicConfig& rgb_intrinsic,
    const RgbCubeEyeExtrinsicConfig& rgb_cubeeye_extrinsic,
    int stride)
{
    if (depth_entry.spec.type != meere::sensor::FrameType::Depth || stride <= 0) {
        return std::nullopt;
    }

    const auto depth = meere::sensor::frame_cast_basic16u(depth_entry.frame);
    if (!depth || !depth->frameData() || depth->frameData()->empty()) {
        return std::nullopt;
    }
    if (!cubeeye_intrinsics || cubeeye_intrinsics->fx <= 0.0F || cubeeye_intrinsics->fy <= 0.0F) {
        return std::nullopt;
    }

    const int depth_width = depth->frameWidth();
    const int depth_height = depth->frameHeight();
    if (depth_width <= 0 || depth_height <= 0) {
        return std::nullopt;
    }

    const auto expected_count = static_cast<std::size_t>(depth_width) * static_cast<std::size_t>(depth_height);
    const auto* depth_values = depth->frameData();
    if (depth_values->size() < expected_count) {
        return std::nullopt;
    }

    const auto* data = depth_values->data();
    const cv::Matx33f depth_to_rgb_rotation =
        rotation_matrix(rgb_cubeeye_extrinsic.roll_deg, rgb_cubeeye_extrinsic.pitch_deg, rgb_cubeeye_extrinsic.yaw_deg);
    const cv::Vec3f depth_to_rgb_translation(rgb_cubeeye_extrinsic.tx_m, rgb_cubeeye_extrinsic.ty_m, rgb_cubeeye_extrinsic.tz_m);
    const float scale_x = static_cast<float>(camera_frame.width) / static_cast<float>(rgb_intrinsic.width);
    const float scale_y = static_cast<float>(camera_frame.height) / static_cast<float>(rgb_intrinsic.height);
    const float depth_fx = cubeeye_intrinsics->fx;
    const float depth_fy = cubeeye_intrinsics->fy;
    const float depth_cx = cubeeye_intrinsics->cx;
    const float depth_cy = cubeeye_intrinsics->cy;

    std::vector<std::uint8_t> bytes;
    bytes.reserve(((expected_count + static_cast<std::size_t>(stride) - 1U) / static_cast<std::size_t>(stride)) * 3U * sizeof(float));
    std::uint64_t point_count = 0;
    for (int y = 0; y < depth_height; y += stride) {
        for (int x = 0; x < depth_width; x += stride) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(depth_width) + static_cast<std::size_t>(x);
            const float value = static_cast<float>(data[index]);
            if (value <= 0.0F) {
                continue;
            }

            const float z_depth = value * 0.001F;
            const float x_depth = ((static_cast<float>(x) - depth_cx) / depth_fx) * z_depth;
            const float y_depth = ((static_cast<float>(y) - depth_cy) / depth_fy) * z_depth;
            const cv::Vec3f depth_point(x_depth, y_depth, z_depth);
            const cv::Vec3f camera_point = depth_to_rgb_rotation * depth_point + depth_to_rgb_translation;
            if (camera_point[2] <= 0.0F) {
                continue;
            }

            const float rgb_xf = ((rgb_intrinsic.fx * (camera_point[0] / camera_point[2])) + rgb_intrinsic.cx) * scale_x;
            const float rgb_yf = ((rgb_intrinsic.fy * (camera_point[1] / camera_point[2])) + rgb_intrinsic.cy) * scale_y;
            if (rgb_xf < 0.0F || rgb_xf >= static_cast<float>(camera_frame.width) || rgb_yf < 0.0F ||
                rgb_yf >= static_cast<float>(camera_frame.height)) {
                continue;
            }

            const std::size_t offset = bytes.size();
            bytes.resize(offset + (3U * sizeof(float)));
            std::uint8_t* output = bytes.data() + offset;
            write_float32_le(output, rgb_xf);
            write_float32_le(output, rgb_yf);
            write_float32_le(output, z_depth);
            ++point_count;
        }
    }
    if (point_count == 0) {
        return std::nullopt;
    }

    return ViewerPayload{
        .name = "projected_depth",
        .kind = "projected_depth",
        .encoding = "projected_depth_xy_depth_f32",
        .width = camera_frame.width,
        .height = camera_frame.height,
        .point_count = point_count,
        .stride = stride,
        .source_timestamp_ms = static_cast<std::uint64_t>(depth_entry.frame->timestamp()),
        .bytes = std::move(bytes),
    };
}

ViewerPayload cubeeye_payload(const CubeEyeFrameEntry& entry, int pointcloud_downsample, const PointCloudRoiConfig& pointcloud_roi)
{
    if (entry.spec.type == meere::sensor::FrameType::PointCloud) {
        return cubeeye_pointcloud_payload(entry, pointcloud_downsample, pointcloud_roi);
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
