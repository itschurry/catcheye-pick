#include "pick/processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/input/pixel_format.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "pick/viewer_payload_builder.hpp"

namespace catcheye::pick {
namespace {

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

RobotPoint transform_robot_point(const RobotTransformConfig& transform, const RobotPoint& point)
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
        .x = (r00 * point.x) + (r01 * point.y) + (r02 * point.z) + transform.translation_m[0],
        .y = (r10 * point.x) + (r11 * point.y) + (r12 * point.z) + transform.translation_m[1],
        .z = (r20 * point.x) + (r21 * point.y) + (r22 * point.z) + transform.translation_m[2],
    };
}

bool finite_point(const RobotPoint& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool finite_pose(const PoseCamera& pose)
{
    return finite_point(pose.translation_m) && std::isfinite(pose.rotation_quat_xyzw[0]) &&
        std::isfinite(pose.rotation_quat_xyzw[1]) && std::isfinite(pose.rotation_quat_xyzw[2]) &&
        std::isfinite(pose.rotation_quat_xyzw[3]);
}

RobotPoint rotate_point_xyzw(const float quat[4], const ObjectPointConfig& point)
{
    const float x = quat[0];
    const float y = quat[1];
    const float z = quat[2];
    const float w = quat[3];

    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    return RobotPoint{
        .x = ((1.0F - (2.0F * (yy + zz))) * point.x) + ((2.0F * (xy - wz)) * point.y) + ((2.0F * (xz + wy)) * point.z),
        .y = ((2.0F * (xy + wz)) * point.x) + ((1.0F - (2.0F * (xx + zz))) * point.y) + ((2.0F * (yz - wx)) * point.z),
        .z = ((2.0F * (xz - wy)) * point.x) + ((2.0F * (yz + wx)) * point.y) + ((1.0F - (2.0F * (xx + yy))) * point.z),
    };
}

std::optional<ObjectPointConfig> pick_point_for_product(
    const std::vector<ProductObjectConfig>& object_catalog,
    const std::string& product_id)
{
    for (const auto& object : object_catalog) {
        if (object.product_id == product_id) {
            return object.pick_point_object_m;
        }
    }
    return std::nullopt;
}

RobotPoint transform_object_point_to_camera(const PoseCamera& pose, const ObjectPointConfig& point_object_m)
{
    const RobotPoint rotated = rotate_point_xyzw(pose.rotation_quat_xyzw, point_object_m);
    return RobotPoint{
        .x = rotated.x + pose.translation_m.x,
        .y = rotated.y + pose.translation_m.y,
        .z = rotated.z + pose.translation_m.z,
    };
}

cv::Mat frame_to_gray(const catcheye::input::Frame& frame)
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
    case catcheye::input::PixelFormat::GRAY8: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        return wrapped.clone();
    }
    case catcheye::input::PixelFormat::NV12: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        return wrapped.clone();
    }
    case catcheye::input::PixelFormat::RGB: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat gray;
        cv::cvtColor(wrapped, gray, cv::COLOR_RGB2GRAY);
        return gray;
    }
    case catcheye::input::PixelFormat::BGR: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat gray;
        cv::cvtColor(wrapped, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    case catcheye::input::PixelFormat::RGBA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat gray;
        cv::cvtColor(wrapped, gray, cv::COLOR_RGBA2GRAY);
        return gray;
    }
    case catcheye::input::PixelFormat::BGRA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat gray;
        cv::cvtColor(wrapped, gray, cv::COLOR_BGRA2GRAY);
        return gray;
    }
    case catcheye::input::PixelFormat::UNKNOWN:
        break;
    }
    return {};
}

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position(
    const catcheye::BoundingBox& box,
    const catcheye::input::Frame& depth_frame,
    const CameraIntrinsicsConfig& intrinsics,
    const DepthProjectionConfig& depth_projection)
{
    const cv::Mat depth_gray = frame_to_gray(depth_frame);
    if (depth_gray.empty()) {
        throw std::runtime_error("failed to convert depth frame for 3D projection");
    }
    if (intrinsics.width <= 0 || intrinsics.height <= 0 || intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F) {
        throw std::runtime_error("camera intrinsics are required for 3D projection");
    }

    const float scale_x = static_cast<float>(depth_gray.cols) / static_cast<float>(intrinsics.width);
    const float scale_y = static_cast<float>(depth_gray.rows) / static_cast<float>(intrinsics.height);
    const int center_x = std::clamp(static_cast<int>(std::lround((box.x + (box.width * 0.5F)) * scale_x)), 0, depth_gray.cols - 1);
    const int center_y = std::clamp(static_cast<int>(std::lround((box.y + (box.height * 0.5F)) * scale_y)), 0, depth_gray.rows - 1);
    const int half_width = std::max(1, static_cast<int>(std::lround(box.width * scale_x * 0.20F)));
    const int half_height = std::max(1, static_cast<int>(std::lround(box.height * scale_y * 0.20F)));
    const int min_px = std::max(0, center_x - half_width);
    const int max_px = std::min(depth_gray.cols - 1, center_x + half_width);
    const int min_py = std::max(0, center_y - half_height);
    const int max_py = std::min(depth_gray.rows - 1, center_y + half_height);

    float depth_sum_m = 0.0F;
    int sample_count = 0;
    for (int y = min_py; y <= max_py; y += 2) {
        const auto* row = depth_gray.ptr<std::uint8_t>(y);
        for (int x = min_px; x <= max_px; x += 2) {
            const float depth_m = static_cast<float>(row[x]) / 255.0F * depth_projection.max_depth_m;
            if (depth_m >= depth_projection.min_depth_m && depth_m <= depth_projection.max_depth_m) {
                depth_sum_m += depth_m;
                ++sample_count;
            }
        }
    }
    if (sample_count == 0) {
        return std::nullopt;
    }

    const float depth_m = depth_sum_m / static_cast<float>(sample_count);
    const float color_x = static_cast<float>(center_x) / scale_x;
    const float color_y = static_cast<float>(center_y) / scale_y;
    const float x_m = (color_x - intrinsics.cx) * depth_m / intrinsics.fx;
    const float y_m = (color_y - intrinsics.cy) * depth_m / intrinsics.fy;
    const float z_m = depth_m;

    const float min_color_x = static_cast<float>(min_px) / scale_x;
    const float max_color_x = static_cast<float>(max_px) / scale_x;
    const float min_color_y = static_cast<float>(min_py) / scale_y;
    const float max_color_y = static_cast<float>(max_py) / scale_y;
    return PickDetectionResult::ObjectPosition{
        .x = x_m,
        .y = y_m,
        .z = z_m,
        .sample_count = sample_count,
        .sample_x = static_cast<int>(std::lround(color_x)),
        .sample_y = static_cast<int>(std::lround(color_y)),
        .min_x = (min_color_x - intrinsics.cx) * depth_m / intrinsics.fx,
        .min_y = (min_color_y - intrinsics.cy) * depth_m / intrinsics.fy,
        .min_z = depth_m,
        .max_x = (max_color_x - intrinsics.cx) * depth_m / intrinsics.fx,
        .max_y = (max_color_y - intrinsics.cy) * depth_m / intrinsics.fy,
        .max_z = depth_m,
    };
}

std::optional<PickCandidate> build_pick_candidate(
    std::uint64_t frame_index,
    int id,
    const PickDetectionResult& detection,
    const RobotCalibrationConfig& robot_calibration)
{
    if (!detection.position.has_value()) {
        return std::nullopt;
    }

    const auto& position = *detection.position;
    const std::string product_id = detection.class_name.empty() ? std::to_string(detection.class_id) : detection.class_name;
    PickCandidate candidate{
        .id = id,
        .object_id = product_id + ":" + std::to_string(frame_index) + ":" + std::to_string(id),
        .product_id = product_id,
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

RobotCalibrationConfig PickProcessor::robot_calibration() const
{
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return config_.robot_calibration;
}

std::vector<PoseEstimate> PickProcessor::pose_estimates() const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return pose_estimates_;
}

bool PickProcessor::update_pose_estimates(std::vector<PoseEstimate> estimates)
{
    const RobotCalibrationConfig robot_calibration = this->robot_calibration();
    for (auto& estimate : estimates) {
        if (estimate.object_id.empty() || estimate.product_id.empty() || !std::isfinite(estimate.confidence) ||
            estimate.confidence < 0.0F || estimate.confidence > 1.0F || !finite_pose(estimate.pose_camera)) {
            return false;
        }
        const float* q = estimate.pose_camera.rotation_quat_xyzw;
        const float quat_norm = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
        if (!std::isfinite(quat_norm) || quat_norm < 0.999F || quat_norm > 1.001F) {
            return false;
        }
        const auto pick_point_object_m = pick_point_for_product(config_.object_catalog, estimate.product_id);
        if (!pick_point_object_m.has_value()) {
            return false;
        }
        estimate.pick_point_camera_m = transform_object_point_to_camera(estimate.pose_camera, *pick_point_object_m);
        if (!finite_point(estimate.pick_point_camera_m)) {
            return false;
        }
        estimate.r1 = std::nullopt;
        estimate.r2 = std::nullopt;
        if (robot_calibration.enabled && estimate.confidence >= robot_calibration.min_confidence) {
            estimate.r1 = transform_robot_point(robot_calibration.r1, estimate.pick_point_camera_m);
            estimate.r2 = transform_robot_point(robot_calibration.r2, estimate.pick_point_camera_m);
        }
    }

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_estimates_ = std::move(estimates);
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
    if (config_.depth_projection.enabled && !frame.depth_visual.has_value()) {
        throw std::runtime_error("3D pick detection requires depth input");
    }

    PickDetectionFrame output;
    output.frame_index = frame.frame_index;
    output.pose_estimates = pose_estimates();
    const catcheye::input::Frame& color_frame = *frame.color;
    const std::vector<catcheye::Detection> detections = detector_->detect(color_frame);
    const RobotCalibrationConfig robot_calibration = this->robot_calibration();
    output.detections.reserve(detections.size());
    output.pick_candidates.reserve(detections.size());
    int candidate_id = 1;
    const catcheye::input::Frame* depth_frame = frame.depth_visual.has_value() ? &*frame.depth_visual : nullptr;
    for (const auto& detection : detections) {
        std::optional<PickDetectionResult::ObjectPosition> position;
        if (config_.depth_projection.enabled) {
            position = estimate_object_position(
                detection.box,
                *depth_frame,
                config_.camera_intrinsics,
                config_.depth_projection);
        }
        output.detections.push_back(PickDetectionResult{
            .class_id = detection.class_id,
            .class_name = detector_->class_name(detection.class_id),
            .score = detection.score,
            .box = detection.box,
            .position = position,
        });
        if (const auto candidate = build_pick_candidate(frame.frame_index, candidate_id, output.detections.back(), robot_calibration)) {
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
    output.payloads.reserve(1U);
    if (frame.color.has_value()) {
        output.payloads.push_back(camera_payload(*frame.color));
    }
    if (frame.depth_visual.has_value()) {
        output.payloads.push_back(depth_payload(*frame.depth_visual));
    }
    return output;
}

} // namespace catcheye::pick
