#include "pick/processor.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CubeEyeBasicFrame.h"
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
                throw std::invalid_argument("--cubeye-frames contains an empty item");
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
        throw std::invalid_argument("--cubeye-frames contains an empty item");
    }
    items.push_back(current);
    return items;
}

CubeEyeFrameSpec parse_cubeye_frame_name(const std::string& name)
{
    if (name == "depth") {
        return {.name = "cubeye_depth", .type = meere::sensor::FrameType::Depth};
    }
    if (name == "amplitude") {
        return {.name = "cubeye_amplitude", .type = meere::sensor::FrameType::Amplitude};
    }
    if (name == "rgb") {
        return {.name = "cubeye_rgb", .type = meere::sensor::FrameType::RGB};
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
        .width = bgr.cols,
        .height = bgr.rows,
        .source_timestamp_ms = static_cast<std::uint64_t>(frame.timestamp),
        .jpeg = encode_jpeg(bgr),
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

ViewerPayload cubeye_payload(const CubeEyeFrameEntry& entry)
{
    cv::Mat bgr;
    if (entry.spec.type == meere::sensor::FrameType::RGB) {
        if (entry.frame->frameDataType() != meere::sensor::DataType::U8) {
            throw std::runtime_error("CubeEye RGB frame is not U8");
        }
        bgr = rgb_frame_to_bgr(entry.frame);
    } else {
        if (entry.frame->frameDataType() != meere::sensor::DataType::U16) {
            throw std::runtime_error("CubeEye " + cubeye_frame_label(entry.spec.type) + " frame is not U16");
        }
        bgr = normalize_u16_frame_to_bgr(entry.frame);
    }
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert CubeEye " + cubeye_frame_label(entry.spec.type) + " frame");
    }

    return ViewerPayload{
        .name = entry.spec.name,
        .kind = cubeye_frame_label(entry.spec.type),
        .width = bgr.cols,
        .height = bgr.rows,
        .source_timestamp_ms = static_cast<std::uint64_t>(entry.frame->timestamp()),
        .jpeg = encode_jpeg(bgr),
    };
}

} // namespace

std::vector<CubeEyeFrameSpec> parse_cubeye_frames(std::string_view value)
{
    std::vector<CubeEyeFrameSpec> specs;
    for (const std::string& item : split_csv(value)) {
        const CubeEyeFrameSpec spec = parse_cubeye_frame_name(item);
        const auto duplicate = std::find_if(specs.begin(), specs.end(), [&](const CubeEyeFrameSpec& existing) {
            return existing.type == spec.type;
        });
        if (duplicate != specs.end()) {
            throw std::invalid_argument("duplicate CubeEye frame: " + item);
        }
        specs.push_back(spec);
    }
    if (specs.empty()) {
        throw std::invalid_argument("--cubeye-frames requires at least one frame");
    }
    return specs;
}

int cubeye_frame_mask(std::span<const CubeEyeFrameSpec> specs)
{
    int mask = 0;
    for (const auto& spec : specs) {
        mask |= static_cast<int>(spec.type);
    }
    return mask;
}

std::string cubeye_frame_label(meere::sensor::FrameType type)
{
    switch (type) {
    case meere::sensor::FrameType::Depth:
        return "depth";
    case meere::sensor::FrameType::Amplitude:
        return "amplitude";
    case meere::sensor::FrameType::RGB:
        return "rgb";
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
    const catcheye::input::Frame& camera_frame,
    const CubeEyeFrameSet& cubeye_frames,
    std::uint64_t frame_index) const
{
    if (config_.detection_enabled) {
        throw std::runtime_error("pick detection pipeline is not implemented yet");
    }

    PickViewerFrame output;
    output.frame_index = frame_index;
    output.payloads.reserve(1U + cubeye_frames.frames.size());
    output.payloads.push_back(camera_payload(camera_frame));
    for (const auto& frame : cubeye_frames.frames) {
        output.payloads.push_back(cubeye_payload(frame));
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
        << "\"payload_encoding\":\"jpeg\","
        << "\"streams\":[";
    for (std::size_t i = 0; i < frame.payloads.size(); ++i) {
        const auto& payload = frame.payloads[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{"
            << "\"name\":\"" << payload.name << "\","
            << "\"kind\":\"" << payload.kind << "\","
            << "\"payload_index\":" << i << ','
            << "\"width\":" << payload.width << ','
            << "\"height\":" << payload.height << ','
            << "\"source_timestamp_ms\":" << payload.source_timestamp_ms << ','
            << "\"payload_size\":" << payload.jpeg.size()
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
        spans.emplace_back(payload.jpeg.data(), payload.jpeg.size());
    }
    return spans;
}

} // namespace catcheye::pick
