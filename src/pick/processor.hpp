#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "catcheye/detection/detector.hpp"
#include "catcheye/input/frame.hpp"
#include "catcheye/roi/camera_roi_config.hpp"
#include "pick/processor_config.hpp"
#include "pick/rgbd_frame.hpp"

namespace catcheye::pick {

struct ViewerPayload {
    std::string name;
    std::string kind;
    std::string encoding = "jpeg";
    int width = 0;
    int height = 0;
    std::uint64_t point_count = 0;
    int stride = 1;
    std::uint64_t source_timestamp_ms = 0;
    std::vector<std::uint8_t> bytes;
};

struct RobotPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct PoseCamera {
    RobotPoint translation_m;
    float rotation_quat_xyzw[4] = {0.0F, 0.0F, 0.0F, 1.0F};
};

struct PoseEstimate {
    std::string object_id;
    std::string product_id;
    float confidence = 0.0F;
    PoseCamera pose_camera;
    RobotPoint pick_point_camera_m;
    std::optional<RobotPoint> r1;
    std::optional<RobotPoint> r2;
};

struct PickCandidate {
    int id = 0;
    std::string object_id;
    std::string product_id;
    float confidence = 0.0F;
    float center_x = 0.0F;
    float center_y = 0.0F;
    float center_z = 0.0F;
    float roll_deg = 0.0F;
    float pitch_deg = 0.0F;
    float yaw_deg = 0.0F;
    float min_x = 0.0F;
    float min_y = 0.0F;
    float min_z = 0.0F;
    float max_x = 0.0F;
    float max_y = 0.0F;
    float max_z = 0.0F;
    float pick_x = 0.0F;
    float pick_y = 0.0F;
    float pick_z = 0.0F;
    std::optional<RobotPoint> r1;
    std::optional<RobotPoint> r2;
};

struct PickViewerFrame {
    std::uint64_t frame_index = 0;
    std::vector<ViewerPayload> payloads;
    bool roi_enabled = false;
    catcheye::roi::CameraRoiConfig roi_config;
    bool pallet_roi_enabled = false;
    catcheye::roi::CameraRoiConfig pallet_roi_config;
};

struct PickDetectionResult {
    int class_id = -1;
    std::string class_name;
    float score = 0.0F;
    catcheye::BoundingBox box{};
    struct ObjectPosition {
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        int sample_count = 0;
        int sample_x = 0;
        int sample_y = 0;
        float min_x = 0.0F;
        float min_y = 0.0F;
        float min_z = 0.0F;
        float max_x = 0.0F;
        float max_y = 0.0F;
        float max_z = 0.0F;
    };
    std::optional<ObjectPosition> position;
};

struct PickDetectionFrame {
    std::uint64_t frame_index = 0;
    std::vector<PickDetectionResult> detections;
    std::vector<PickCandidate> pick_candidates;
    std::vector<PoseEstimate> pose_estimates;
};

class PickProcessor final {
  public:
    explicit PickProcessor(PickProcessorConfig config);

    bool initialize();
    PickDetectionFrame process_detection_frame(const RgbdFrame& frame);
    PickViewerFrame process_viewer_frame(const RgbdFrame& frame) const;
    RobotCalibrationConfig robot_calibration() const;
    std::vector<PoseEstimate> pose_estimates() const;
    bool update_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);
    bool update_pallet_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);
    bool update_robot_calibration(RobotCalibrationConfig config);
    bool update_pose_estimates(std::vector<PoseEstimate> estimates);

  private:
    struct RoiSnapshot {
        bool enabled = false;
        catcheye::roi::CameraRoiConfig config;
    };

    RoiSnapshot roi_snapshot() const;
    RoiSnapshot pallet_roi_snapshot() const;

    mutable std::mutex roi_mutex_;
    mutable std::mutex pose_mutex_;
    PickProcessorConfig config_;
    std::vector<PoseEstimate> pose_estimates_;
    std::unique_ptr<catcheye::IDetector> detector_;
};

} // namespace catcheye::pick
