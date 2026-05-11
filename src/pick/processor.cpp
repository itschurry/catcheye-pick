#include "pick/processor.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "catcheye/input/pixel_format.hpp"

namespace catcheye::pick {
namespace {

std::uint64_t wall_clock_millis()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::vector<std::string> split_csv(std::string_view value)
{
    std::vector<std::string> items;
    std::string current;
    for (const char ch : value) {
        if (ch == ',') {
            if (current.empty()) {
                throw std::invalid_argument("--cubeeye-frames contains an empty item");
            }
            items.push_back(current);
            current.clear();
            continue;
        }
        if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') {
            current.push_back(ch);
        }
    }
    if (current.empty()) {
        throw std::invalid_argument("--cubeeye-frames contains an empty item");
    }
    items.push_back(current);
    return items;
}

CubeEyeFrameSpec parse_cubeeye_frame_name(const std::string& name)
{
    if (name == "depth") {
        return {.name = "cubeeye_depth", .type = meere::sensor::FrameType::Depth};
    }
    if (name == "amplitude") {
        return {.name = "cubeeye_amplitude", .type = meere::sensor::FrameType::Amplitude};
    }
    if (name == "rgb") {
        return {.name = "cubeeye_rgb", .type = meere::sensor::FrameType::RGB};
    }
    if (name == "pointcloud") {
        return {.name = "cubeeye_pointcloud", .type = meere::sensor::FrameType::PointCloud};
    }
    throw std::invalid_argument("unsupported CubeEye frame: " + name);
}

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

ViewerPayload camera_payload(const catcheye::input::Frame& frame)
{
    const cv::Mat bgr = frame_to_bgr(frame);
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

void append_float32_le(std::vector<std::uint8_t>& output, float value)
{
    const auto bits = std::bit_cast<std::uint32_t>(value);
    output.push_back(static_cast<std::uint8_t>(bits & 0xFFU));
    output.push_back(static_cast<std::uint8_t>((bits >> 8U) & 0xFFU));
    output.push_back(static_cast<std::uint8_t>((bits >> 16U) & 0xFFU));
    output.push_back(static_cast<std::uint8_t>((bits >> 24U) & 0xFFU));
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
    bytes.reserve(static_cast<std::size_t>(point_count) * 3U * sizeof(float));

    for (int y = 0; y < height; y += downsample) {
        for (int x = 0; x < width; x += downsample) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            append_float32_le(bytes, xs->at(index));
            append_float32_le(bytes, ys->at(index));
            append_float32_le(bytes, zs->at(index));
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

} // namespace

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value)
{
    std::vector<CubeEyeFrameSpec> specs;
    for (const std::string& item : split_csv(value)) {
        const CubeEyeFrameSpec spec = parse_cubeeye_frame_name(item);
        const auto duplicate = std::find_if(specs.begin(), specs.end(), [&](const CubeEyeFrameSpec& existing) {
            return existing.type == spec.type;
        });
        if (duplicate != specs.end()) {
            throw std::invalid_argument("duplicate CubeEye frame: " + item);
        }
        specs.push_back(spec);
    }
    if (specs.empty()) {
        throw std::invalid_argument("--cubeeye-frames requires at least one frame");
    }
    return specs;
}

int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs)
{
    int mask = 0;
    for (const auto& spec : specs) {
        mask |= static_cast<int>(spec.type);
    }
    return mask;
}

std::string cubeeye_frame_label(meere::sensor::FrameType type)
{
    switch (type) {
    case meere::sensor::FrameType::Depth:
        return "depth";
    case meere::sensor::FrameType::Amplitude:
        return "amplitude";
    case meere::sensor::FrameType::RGB:
        return "rgb";
    case meere::sensor::FrameType::PointCloud:
        return "pointcloud";
    default:
        return "unknown";
    }
}

PickProcessor::PickProcessor(PickProcessorConfig config)
    : config_(std::move(config)) {}

bool PickProcessor::initialize()
{
    return true;
}

PickViewerFrame PickProcessor::process_viewer_frame(
    const std::optional<catcheye::input::Frame>& camera_frame,
    const CubeEyeFrameSet& cubeeye_frames,
    std::uint64_t frame_index) const
{
    if (config_.detection_enabled) {
        throw std::runtime_error("pick detection pipeline is not implemented yet");
    }

    PickViewerFrame output;
    output.frame_index = frame_index;
    output.payloads.reserve(1U + cubeeye_frames.frames.size());
    if (camera_frame.has_value()) {
        output.payloads.push_back(camera_payload(*camera_frame));
    }
    for (const auto& frame : cubeeye_frames.frames) {
        output.payloads.push_back(cubeeye_payload(frame, config_.pointcloud_downsample));
    }
    return output;
}

std::string build_viewer_metadata(const PickViewerFrame& frame)
{
    std::ostringstream oss;
    oss << "{\"type\":\"viewer_frame\","
        << "\"viewer_only\":true,"
        << "\"frame_index\":" << frame.frame_index << ','
        << "\"wall_timestamp_ms\":" << wall_clock_millis() << ','
        << "\"streams\":[";
    for (std::size_t i = 0; i < frame.payloads.size(); ++i) {
        const auto& payload = frame.payloads[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{"
            << "\"name\":\"" << payload.name << "\","
            << "\"kind\":\"" << payload.kind << "\","
            << "\"encoding\":\"" << payload.encoding << "\","
            << "\"payload_index\":" << i << ','
            << "\"width\":" << payload.width << ','
            << "\"height\":" << payload.height << ','
            << "\"point_count\":" << payload.point_count << ','
            << "\"stride\":" << payload.stride << ','
            << "\"source_timestamp_ms\":" << payload.source_timestamp_ms << ','
            << "\"payload_size\":" << payload.bytes.size()
            << "}";
    }
    oss << "]}";
    return oss.str();
}

std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame)
{
    std::vector<std::span<const std::uint8_t>> spans;
    spans.reserve(frame.payloads.size());
    for (const auto& payload : frame.payloads) {
        spans.emplace_back(payload.bytes.data(), payload.bytes.size());
    }
    return spans;
}

} // namespace catcheye::pick
