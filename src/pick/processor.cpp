#include "pick/processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "pick/object_position_estimator.hpp"
#include "pick/viewer_payload_builder.hpp"

namespace catcheye::pick {
namespace {

const CubeEyeFrameEntry* find_pointcloud_frame(const CubeEyeFrameSet& frame_set)
{
    const auto it = std::find_if(frame_set.frames.begin(), frame_set.frames.end(),
                                 [](const CubeEyeFrameEntry& entry) { return entry.spec.type == meere::sensor::FrameType::PointCloud; });
    return it == frame_set.frames.end() ? nullptr : &*it;
}

const CubeEyeFrameEntry* find_depth_frame(const CubeEyeFrameSet& frame_set)
{
    const auto it = std::find_if(frame_set.frames.begin(), frame_set.frames.end(),
                                 [](const CubeEyeFrameEntry& entry) { return entry.spec.type == meere::sensor::FrameType::Depth; });
    return it == frame_set.frames.end() ? nullptr : &*it;
}

RobotPoint transform_pick_point(const RobotTransformConfig& transform, const PickCandidate& candidate)
{
    constexpr float pi = 3.14159265358979323846F;
    const float roll = transform.rotation_rpy_deg[0] * pi / 180.0F;
    const float pitch = transform.rotation_rpy_deg[1] * pi / 180.0F;
    const float yaw = transform.rotation_rpy_deg[2] * pi / 180.0F;
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float r00 = cy * cp;
    const float r01 = (cy * sp * sr) - (sy * cr);
    const float r02 = (cy * sp * cr) + (sy * sr);
    const float r10 = sy * cp;
    const float r11 = (sy * sp * sr) + (cy * cr);
    const float r12 = (sy * sp * cr) - (cy * sr);
    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    return RobotPoint{
        .x = (r00 * candidate.pick_x) + (r01 * candidate.pick_y) + (r02 * candidate.pick_z) + transform.translation_m[0],
        .y = (r10 * candidate.pick_x) + (r11 * candidate.pick_y) + (r12 * candidate.pick_z) + transform.translation_m[1],
        .z = (r20 * candidate.pick_x) + (r21 * candidate.pick_y) + (r22 * candidate.pick_z) + transform.translation_m[2],
    };
}

std::optional<PickCandidate> build_pick_candidate(
    int id,
    const PickDetectionResult& detection,
    const RobotCalibrationConfig& robot_calibration)
{
    if (!detection.position.has_value()) {
        return std::nullopt;
    }

    const auto& position = *detection.position;
    PickCandidate candidate{
        .id = id,
        .product_id = detection.class_name,
        .confidence = detection.score,
        .center_x = position.x,
        .center_y = position.y,
        .center_z = position.z,
        .roll_deg = 0.0F,
        .pitch_deg = 0.0F,
        .yaw_deg = 0.0F,
        .min_x = position.min_x,
        .min_y = position.min_y,
        .min_z = position.min_z,
        .max_x = position.max_x,
        .max_y = position.max_y,
        .max_z = position.max_z,
        .pick_x = position.x,
        .pick_y = position.y,
        .pick_z = position.z,
        .r1 = std::nullopt,
        .r2 = std::nullopt,
    };
    if (robot_calibration.enabled && candidate.confidence >= robot_calibration.min_confidence) {
        candidate.r1 = transform_pick_point(robot_calibration.r1, candidate);
        candidate.r2 = transform_pick_point(robot_calibration.r2, candidate);
    }
    return candidate;
}

} // namespace

PickProcessor::PickProcessor(PickProcessorConfig config)
    : config_(std::move(config)), detector_(config_.detection_enabled ? catcheye::create_detector(config_.detector) : nullptr)
{}

bool PickProcessor::initialize()
{
    if (config_.detection_enabled && !detector_->initialize()) {
        return false;
    }
    return true;
}

PickProcessor::RoiSnapshot PickProcessor::roi_snapshot() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return RoiSnapshot{
        .enabled = config_.roi_enabled,
        .config = config_.roi_config,
    };
}

PickProcessor::RoiSnapshot PickProcessor::pallet_roi_snapshot() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return RoiSnapshot{
        .enabled = config_.pallet_roi_enabled,
        .config = config_.pallet_roi_config,
    };
}

bool PickProcessor::update_roi_config(const catcheye::roi::CameraRoiConfig& roi_config)
{
    const auto validation = catcheye::roi::validate_camera_roi_config(roi_config);
    if (!validation.valid) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.roi_enabled = true;
    config_.roi_config = roi_config;
    return true;
}

bool PickProcessor::update_pallet_roi_config(const catcheye::roi::CameraRoiConfig& roi_config)
{
    const auto validation = catcheye::roi::validate_camera_roi_config(roi_config);
    if (!validation.valid) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.pallet_roi_enabled = true;
    config_.pallet_roi_config = roi_config;
    return true;
}

RgbCubeEyeOffset PickProcessor::rgb_cubeeye_offset() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.rgb_cubeeye_offset;
}

PointCloudRoiConfig PickProcessor::pointcloud_roi_config() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.pointcloud_roi_config;
}

RobotCalibrationConfig PickProcessor::robot_calibration() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.robot_calibration;
}

bool PickProcessor::update_rgb_cubeeye_offset(RgbCubeEyeOffset offset)
{
    if (!std::isfinite(offset.u) || !std::isfinite(offset.v) || offset.u < -1.0F || offset.u > 1.0F || offset.v < -1.0F ||
        offset.v > 1.0F) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.rgb_cubeeye_offset = offset;
    return true;
}

bool PickProcessor::update_pointcloud_roi_config(PointCloudRoiConfig config)
{
    if (!std::isfinite(config.min_x_m) || !std::isfinite(config.max_x_m) || !std::isfinite(config.min_y_m) ||
        !std::isfinite(config.max_y_m) || !std::isfinite(config.min_z_m) || !std::isfinite(config.max_z_m) ||
        config.max_x_m <= config.min_x_m || config.max_y_m <= config.min_y_m || config.max_z_m <= config.min_z_m) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.pointcloud_roi_config = config;
    return true;
}

bool PickProcessor::update_robot_calibration(RobotCalibrationConfig config)
{
    for (const auto* transform : {&config.r1, &config.r2}) {
        for (float value : transform->translation_m) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        for (float value : transform->rotation_rpy_deg) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
    }
    if (!std::isfinite(config.min_confidence) || config.min_confidence < 0.0F || config.min_confidence > 1.0F) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.robot_calibration = config;
    return true;
}

PickDetectionFrame PickProcessor::process_detection_frame(const RgbdFrame& frame)
{
    if (!config_.detection_enabled || !detector_) {
        throw std::runtime_error("pick detection pipeline is disabled");
    }
    if (!frame.color.has_value()) {
        throw std::runtime_error("pick detection requires color input");
    }

    PickDetectionFrame output;
    output.frame_index = frame.frame_index;
    const catcheye::input::Frame& color_frame = *frame.color;
    const std::vector<catcheye::Detection> detections = detector_->detect(color_frame);
    const CubeEyeFrameEntry* pointcloud_frame = find_pointcloud_frame(frame.depth);
    const CubeEyeFrameEntry* depth_frame = find_depth_frame(frame.depth);
    const CubeEyeFrameEntry* position_frame = pointcloud_frame != nullptr ? pointcloud_frame : depth_frame;
    const RgbCubeEyeOffset rgb_cubeeye_offset = this->rgb_cubeeye_offset();
    const RobotCalibrationConfig robot_calibration = this->robot_calibration();
    output.detections.reserve(detections.size());
    output.pick_candidates.reserve(detections.size());
    int candidate_id = 1;
    for (const auto& detection : detections) {
        std::optional<PickDetectionResult::ObjectPosition> position;
        if (position_frame != nullptr) {
            position = estimate_object_position(detection.box, color_frame, *position_frame, frame.depth.intrinsics, rgb_cubeeye_offset);
        }
        output.detections.push_back(PickDetectionResult{
            .class_id = detection.class_id,
            .class_name = detector_->class_name(detection.class_id),
            .score = detection.score,
            .box = detection.box,
            .position = position,
        });
        if (const auto candidate = build_pick_candidate(candidate_id, output.detections.back(), robot_calibration)) {
            output.pick_candidates.push_back(*candidate);
            ++candidate_id;
        }
    }
    return output;
}

PickViewerFrame PickProcessor::process_viewer_frame(const RgbdFrame& frame) const
{
    PickViewerFrame output;
    output.frame_index = frame.frame_index;
    const RoiSnapshot roi = roi_snapshot();
    const RoiSnapshot pallet_roi = pallet_roi_snapshot();
    output.roi_enabled = roi.enabled;
    output.roi_config = roi.config;
    output.pallet_roi_enabled = pallet_roi.enabled;
    output.pallet_roi_config = pallet_roi.config;
    output.payloads.reserve(1U + frame.depth.frames.size());
    if (frame.color.has_value()) {
        output.payloads.push_back(camera_payload(*frame.color));
    }
    const PointCloudRoiConfig pointcloud_roi = pointcloud_roi_config();
    for (const auto& depth_frame : frame.depth.frames) {
        output.payloads.push_back(cubeeye_payload(depth_frame, config_.pointcloud_downsample, pointcloud_roi));
    }
    return output;
}

} // namespace catcheye::pick
