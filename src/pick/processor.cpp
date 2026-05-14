#include "pick/processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "pick/object_position_estimator.hpp"
#include "pick/pallet_candidate_detector.hpp"
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

PalletCandidateConfig PickProcessor::pallet_candidate_config() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.pallet_candidate_config;
}

RobotCalibrationConfig PickProcessor::robot_calibration() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.robot_calibration;
}

PointCloudRoiConfig PickProcessor::pointcloud_roi_config() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.pointcloud_roi_config;
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

bool PickProcessor::update_pallet_candidate_config(PalletCandidateConfig config)
{
    if (!std::isfinite(config.min_height_m) || !std::isfinite(config.max_height_m) || config.min_height_m <= 0.0F ||
        config.max_height_m <= config.min_height_m || config.min_points <= 0 || config.max_image_gap_px < 1) {
        return false;
    }

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.pallet_candidate_config = config;
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

    std::lock_guard<std::mutex> lock(roi_mutex_);
    config_.robot_calibration = config;
    return true;
}

PickDetectionFrame PickProcessor::process_detection_frame(const catcheye::input::Frame& camera_frame,
                                                          const CubeEyeFrameSet& cubeeye_frames,
                                                          std::uint64_t frame_index)
{
    if (!config_.detection_enabled || !detector_) {
        throw std::runtime_error("pick detection pipeline is disabled");
    }

    PickDetectionFrame output;
    output.frame_index = frame_index;
    const std::vector<catcheye::Detection> detections = detector_->detect(camera_frame);
    const CubeEyeFrameEntry* pointcloud_frame = find_pointcloud_frame(cubeeye_frames);
    const CubeEyeFrameEntry* depth_frame = find_depth_frame(cubeeye_frames);
    const CubeEyeFrameEntry* position_frame = pointcloud_frame != nullptr ? pointcloud_frame : depth_frame;
    const RgbCubeEyeOffset rgb_cubeeye_offset = this->rgb_cubeeye_offset();
    output.detections.reserve(detections.size());
    for (const auto& detection : detections) {
        std::optional<PickDetectionResult::ObjectPosition> position;
        if (position_frame != nullptr) {
            position = estimate_object_position(detection.box, camera_frame, *position_frame, cubeeye_frames.intrinsics, rgb_cubeeye_offset);
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
                                                    const CubeEyeFrameSet& cubeeye_frames,
                                                    std::uint64_t frame_index) const
{
    PickViewerFrame output;
    output.frame_index = frame_index;
    const RoiSnapshot roi = roi_snapshot();
    const RoiSnapshot pallet_roi = pallet_roi_snapshot();
    output.roi_enabled = roi.enabled;
    output.roi_config = roi.config;
    output.pallet_roi_enabled = pallet_roi.enabled;
    output.pallet_roi_config = pallet_roi.config;
    output.robot_calibration_enabled = robot_calibration().enabled;
    output.payloads.reserve(1U + cubeeye_frames.frames.size());
    if (camera_frame.has_value()) {
        output.payloads.push_back(camera_payload(*camera_frame));
    }
    const PointCloudRoiConfig pointcloud_roi = pointcloud_roi_config();
    for (const auto& frame : cubeeye_frames.frames) {
        output.payloads.push_back(cubeeye_payload(frame, config_.pointcloud_downsample, pointcloud_roi));
    }
    const CubeEyeFrameEntry* pointcloud_frame = find_pointcloud_frame(cubeeye_frames);
    if (pointcloud_frame != nullptr && pallet_roi.enabled) {
        const auto candidates = detect_pallet_candidates(
            *pointcloud_frame,
            pallet_roi.config,
            pointcloud_roi,
            pallet_candidate_config(),
            robot_calibration());
        output.pallet_plane = candidates.plane;
        output.pallet_candidates = candidates.candidates;
    }
    return output;
}

} // namespace catcheye::pick
