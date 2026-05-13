#include "pick/processor.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/input/pixel_format.hpp"
#include "catcheye/roi/roi_repository.hpp"
#include "catcheye/roi/roi_validation.hpp"

namespace catcheye::pick {
namespace {

constexpr float RGB_TO_CUBEEYE_OFFSET_U = 0.00F;
constexpr float RGB_TO_CUBEEYE_OFFSET_V = 0.40F;
constexpr int WEBSOCKET_CAMERA_MAX_WIDTH = 1280;
constexpr int WEBSOCKET_CAMERA_MAX_HEIGHT = 720;

std::uint64_t wall_clock_millis() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

std::vector<std::string> split_csv(std::string_view value) {
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

CubeEyeFrameSpec parse_cubeeye_frame_name(const std::string& name) {
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

cv::Mat frame_to_bgr(const catcheye::input::Frame& frame) {
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

std::vector<std::uint8_t> encode_jpeg(const cv::Mat& bgr) {
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

cv::Mat limit_camera_payload_resolution(const cv::Mat& bgr) {
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

std::string escape_json(std::string_view value) {
    std::ostringstream oss;
    for (const char ch : value) {
        switch (ch) {
        case '"':
            oss << "\\\"";
            break;
        case '\\':
            oss << "\\\\";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << ch;
            break;
        }
    }
    return oss.str();
}

ViewerPayload camera_payload(const catcheye::input::Frame& frame) {
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

cv::Mat normalize_u16_frame_to_bgr(const meere::sensor::sptr_frame& frame) {
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

cv::Mat rgb_frame_to_bgr(const meere::sensor::sptr_frame& frame) {
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

void write_float32_le(std::uint8_t*& output, float value) {
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

const CubeEyeFrameEntry* find_pointcloud_frame(const CubeEyeFrameSet& frame_set) {
    const auto it = std::find_if(frame_set.frames.begin(), frame_set.frames.end(),
                                 [](const CubeEyeFrameEntry& entry) { return entry.spec.type == meere::sensor::FrameType::PointCloud; });
    return it == frame_set.frames.end() ? nullptr : &*it;
}

struct PointSample {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

float percentile_value(std::vector<float>& values, float percentile) {
    const auto index = static_cast<std::size_t>(std::clamp(percentile, 0.0F, 1.0F) * static_cast<float>(values.size() - 1U));
    const auto it = values.begin() + static_cast<std::ptrdiff_t>(index);
    std::nth_element(values.begin(), it, values.end());
    return *it;
}

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position(const catcheye::BoundingBox& box,
                                                                            const catcheye::input::Frame& camera_frame,
                                                                            const CubeEyeFrameEntry& pointcloud_entry) {
    const auto pointcloud = meere::sensor::frame_cast_pcl32f(pointcloud_entry.frame);
    if (!pointcloud || !pointcloud->frameDataX() || !pointcloud->frameDataY() || !pointcloud->frameDataZ()) {
        return std::nullopt;
    }

    const int width = pointcloud->frameWidth();
    const int height = pointcloud->frameHeight();
    if (width <= 0 || height <= 0 || camera_frame.width <= 0 || camera_frame.height <= 0) {
        return std::nullopt;
    }

    const auto expected_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const auto* xs = pointcloud->frameDataX();
    const auto* ys = pointcloud->frameDataY();
    const auto* zs = pointcloud->frameDataZ();
    if (xs->size() < expected_count || ys->size() < expected_count || zs->size() < expected_count) {
        return std::nullopt;
    }

    const float center_x = box.x + (box.width * 0.5F);
    const float center_y = box.y + (box.height * 0.5F);
    const float pointcloud_u = std::clamp((center_x / static_cast<float>(camera_frame.width)) + RGB_TO_CUBEEYE_OFFSET_U, 0.0F, 1.0F);
    const float pointcloud_v = std::clamp((center_y / static_cast<float>(camera_frame.height)) + RGB_TO_CUBEEYE_OFFSET_V, 0.0F, 1.0F);
    const int pointcloud_x = std::clamp(static_cast<int>(pointcloud_u * static_cast<float>(width)), 0, width - 1);
    const int pointcloud_y = std::clamp(static_cast<int>(pointcloud_v * static_cast<float>(height)), 0, height - 1);

    const float left_u = std::clamp((box.x / static_cast<float>(camera_frame.width)) + RGB_TO_CUBEEYE_OFFSET_U, 0.0F, 1.0F);
    const float top_v = std::clamp((box.y / static_cast<float>(camera_frame.height)) + RGB_TO_CUBEEYE_OFFSET_V, 0.0F, 1.0F);
    const float right_u = std::clamp(((box.x + box.width) / static_cast<float>(camera_frame.width)) + RGB_TO_CUBEEYE_OFFSET_U, 0.0F, 1.0F);
    const float bottom_v =
        std::clamp(((box.y + box.height) / static_cast<float>(camera_frame.height)) + RGB_TO_CUBEEYE_OFFSET_V, 0.0F, 1.0F);
    const int min_px = std::clamp(static_cast<int>(std::floor(std::min(left_u, right_u) * static_cast<float>(width))), 0, width - 1);
    const int max_px = std::clamp(static_cast<int>(std::ceil(std::max(left_u, right_u) * static_cast<float>(width))), 0, width - 1);
    const int min_py = std::clamp(static_cast<int>(std::floor(std::min(top_v, bottom_v) * static_cast<float>(height))), 0, height - 1);
    const int max_py = std::clamp(static_cast<int>(std::ceil(std::max(top_v, bottom_v) * static_cast<float>(height))), 0, height - 1);

    std::vector<PointSample> samples;
    std::vector<float> sample_zs;
    samples.reserve(static_cast<std::size_t>(std::max(0, max_px - min_px + 1)) *
                    static_cast<std::size_t>(std::max(0, max_py - min_py + 1)));

    const auto* x_data = xs->data();
    const auto* y_data = ys->data();
    const auto* z_data = zs->data();
    for (int y = min_py; y <= max_py; ++y) {
        for (int x = min_px; x <= max_px; ++x) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            const float px = x_data[index];
            const float py = y_data[index];
            const float pz = z_data[index];
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) || pz <= 0.0F) {
                continue;
            }
            samples.push_back(PointSample{.x = px, .y = py, .z = pz});
            sample_zs.push_back(pz);
        }
    }

    if (sample_zs.empty()) {
        return std::nullopt;
    }

    const float min_depth = percentile_value(sample_zs, 0.10F);
    const float max_depth = percentile_value(sample_zs, 0.90F);
    float sum_x = 0.0F;
    float sum_y = 0.0F;
    float sum_z = 0.0F;
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    int filtered_count = 0;
    for (const auto& sample : samples) {
        if (sample.z < min_depth || sample.z > max_depth) {
            continue;
        }
        sum_x += sample.x;
        sum_y += sample.y;
        sum_z += sample.z;
        min_x = std::min(min_x, sample.x);
        min_y = std::min(min_y, sample.y);
        min_z = std::min(min_z, sample.z);
        max_x = std::max(max_x, sample.x);
        max_y = std::max(max_y, sample.y);
        max_z = std::max(max_z, sample.z);
        ++filtered_count;
    }

    if (filtered_count == 0) {
        return std::nullopt;
    }

    const float inv_count = 1.0F / static_cast<float>(filtered_count);
    return PickDetectionResult::ObjectPosition{
        .x = sum_x * inv_count,
        .y = sum_y * inv_count,
        .z = sum_z * inv_count,
        .sample_count = filtered_count,
        .pointcloud_x = pointcloud_x,
        .pointcloud_y = pointcloud_y,
        .min_x = min_x,
        .min_y = min_y,
        .min_z = min_z,
        .max_x = max_x,
        .max_y = max_y,
        .max_z = max_z,
    };
}

ViewerPayload cubeeye_pointcloud_payload(const CubeEyeFrameEntry& entry, int downsample) {
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

ViewerPayload cubeeye_payload(const CubeEyeFrameEntry& entry, int pointcloud_downsample) {
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

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value) {
    std::vector<CubeEyeFrameSpec> specs;
    bool has_depth = false;
    bool has_pointcloud = false;
    for (const std::string& item : split_csv(value)) {
        const CubeEyeFrameSpec spec = parse_cubeeye_frame_name(item);
        const auto duplicate =
            std::find_if(specs.begin(), specs.end(), [&](const CubeEyeFrameSpec& existing) { return existing.type == spec.type; });
        if (duplicate != specs.end()) {
            throw std::invalid_argument("duplicate CubeEye frame: " + item);
        }
        has_depth = has_depth || spec.type == meere::sensor::FrameType::Depth;
        has_pointcloud = has_pointcloud || spec.type == meere::sensor::FrameType::PointCloud;
        specs.push_back(spec);
    }
    if (specs.empty()) {
        throw std::invalid_argument("--cubeeye-frames requires at least one frame");
    }
    if (has_depth && has_pointcloud) {
        throw std::invalid_argument("CubeEye depth and pointcloud cannot be selected together");
    }
    return specs;
}

int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs) {
    int mask = 0;
    for (const auto& spec : specs) {
        mask |= static_cast<int>(spec.type);
    }
    return mask;
}

std::string cubeeye_frame_label(meere::sensor::FrameType type) {
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
    : config_(std::move(config)), detector_(config_.detection_enabled ? catcheye::create_detector(config_.detector) : nullptr) {}

bool PickProcessor::initialize() {
    if (config_.detection_enabled && !detector_->initialize()) {
        return false;
    }
    return true;
}

PickProcessor::RoiSnapshot PickProcessor::roi_snapshot() const {
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return RoiSnapshot{
        .enabled = config_.roi_enabled,
        .config = config_.roi_config,
    };
}

PickProcessor::RoiSnapshot PickProcessor::pallet_roi_snapshot() const {
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return RoiSnapshot{
        .enabled = config_.pallet_roi_enabled,
        .config = config_.pallet_roi_config,
    };
}

bool PickProcessor::update_roi_config(const catcheye::roi::CameraRoiConfig& roi_config) {
    const auto validation = catcheye::roi::validate_camera_roi_config(roi_config);
    if (!validation.valid) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.roi_enabled = true;
    config_.roi_config = roi_config;
    return true;
}

bool PickProcessor::update_pallet_roi_config(const catcheye::roi::CameraRoiConfig& roi_config) {
    const auto validation = catcheye::roi::validate_camera_roi_config(roi_config);
    if (!validation.valid) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.pallet_roi_enabled = true;
    config_.pallet_roi_config = roi_config;
    return true;
}

PickDetectionFrame PickProcessor::process_detection_frame(const catcheye::input::Frame& camera_frame, const CubeEyeFrameSet& cubeeye_frames,
                                                          std::uint64_t frame_index) {
    if (!config_.detection_enabled || !detector_) {
        throw std::runtime_error("pick detection pipeline is disabled");
    }

    PickDetectionFrame output;
    output.frame_index = frame_index;
    const std::vector<catcheye::Detection> detections = detector_->detect(camera_frame);
    const CubeEyeFrameEntry* pointcloud_frame = find_pointcloud_frame(cubeeye_frames);
    output.detections.reserve(detections.size());
    for (const auto& detection : detections) {
        std::optional<PickDetectionResult::ObjectPosition> position;
        if (pointcloud_frame != nullptr) {
            position = estimate_object_position(detection.box, camera_frame, *pointcloud_frame);
        }
        output.detections.push_back(PickDetectionResult{
            .class_id = detection.class_id,
            .class_name = detector_->class_name(detection.class_id),
            .score = detection.score,
            .box = detection.box,
            .position = position,
        });
    }
    return output;
}

PickViewerFrame PickProcessor::process_viewer_frame(const std::optional<catcheye::input::Frame>& camera_frame,
                                                    const CubeEyeFrameSet& cubeeye_frames, std::uint64_t frame_index) const {
    PickViewerFrame output;
    output.frame_index = frame_index;
    const RoiSnapshot roi = roi_snapshot();
    const RoiSnapshot pallet_roi = pallet_roi_snapshot();
    output.roi_enabled = roi.enabled;
    output.roi_config = roi.config;
    output.pallet_roi_enabled = pallet_roi.enabled;
    output.pallet_roi_config = pallet_roi.config;
    output.payloads.reserve(1U + cubeeye_frames.frames.size());
    if (camera_frame.has_value()) {
        output.payloads.push_back(camera_payload(*camera_frame));
    }
    for (const auto& frame : cubeeye_frames.frames) {
        output.payloads.push_back(cubeeye_payload(frame, config_.pointcloud_downsample));
    }
    return output;
}

void append_detection_fields(std::ostringstream& oss, const PickDetectionFrame& frame) {
    oss << "\"detection_count\":" << frame.detections.size() << ",\"detections\":[";
    for (std::size_t i = 0; i < frame.detections.size(); ++i) {
        const auto& detection = frame.detections[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{\"class_id\":" << detection.class_id << ",\"class_name\":\"" << escape_json(detection.class_name) << "\""
            << ",\"score\":" << detection.score << ",\"box\":{\"x\":" << detection.box.x << ",\"y\":" << detection.box.y
            << ",\"width\":" << detection.box.width << ",\"height\":" << detection.box.height << "},\"position\":";
        if (detection.position.has_value()) {
            const auto& position = *detection.position;
            oss << "{\"x\":" << position.x << ",\"y\":" << position.y << ",\"z\":" << position.z
                << ",\"sample_count\":" << position.sample_count << ",\"pointcloud_x\":" << position.pointcloud_x
                << ",\"pointcloud_y\":" << position.pointcloud_y << ",\"bbox3d\":{\"min_x\":" << position.min_x
                << ",\"min_y\":" << position.min_y << ",\"min_z\":" << position.min_z << ",\"max_x\":" << position.max_x
                << ",\"max_y\":" << position.max_y << ",\"max_z\":" << position.max_z << "}}";
        } else {
            oss << "null";
        }
        oss << "}";
    }
    oss << "]";
}

std::string build_viewer_metadata(const PickViewerFrame& frame, bool viewer_only, const PickDetectionFrame* detection_frame) {
    std::ostringstream oss;
    oss << "{\"type\":\"viewer_frame\","
        << "\"viewer_only\":" << (viewer_only ? "true" : "false") << ',' << "\"frame_index\":" << frame.frame_index << ','
        << "\"wall_timestamp_ms\":" << wall_clock_millis() << ',' << "\"roi_enabled\":" << (frame.roi_enabled ? "true" : "false") << ','
        << "\"roi\":" << catcheye::roi::RoiRepository::to_json_string(frame.roi_config, 0) << ','
        << "\"pallet_roi_enabled\":" << (frame.pallet_roi_enabled ? "true" : "false") << ','
        << "\"pallet_roi\":" << catcheye::roi::RoiRepository::to_json_string(frame.pallet_roi_config, 0) << ',';
    if (detection_frame != nullptr) {
        append_detection_fields(oss, *detection_frame);
        oss << ',';
    }
    oss << "\"streams\":[";
    for (std::size_t i = 0; i < frame.payloads.size(); ++i) {
        const auto& payload = frame.payloads[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{"
            << "\"name\":\"" << payload.name << "\","
            << "\"kind\":\"" << payload.kind << "\","
            << "\"encoding\":\"" << payload.encoding << "\","
            << "\"payload_index\":" << i << ',' << "\"width\":" << payload.width << ',' << "\"height\":" << payload.height << ','
            << "\"point_count\":" << payload.point_count << ',' << "\"stride\":" << payload.stride << ','
            << "\"source_timestamp_ms\":" << payload.source_timestamp_ms << ',' << "\"payload_size\":" << payload.bytes.size() << "}";
    }
    oss << "]}";
    return oss.str();
}

std::string build_detection_metadata(const PickDetectionFrame& frame) {
    std::ostringstream oss;
    oss << "{\"viewer_only\":false,";
    append_detection_fields(oss, frame);
    oss << "}";
    return oss.str();
}

std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame) {
    std::vector<std::span<const std::uint8_t>> spans;
    spans.reserve(frame.payloads.size());
    for (const auto& payload : frame.payloads) {
        spans.emplace_back(payload.bytes.data(), payload.bytes.size());
    }
    return spans;
}

} // namespace catcheye::pick
