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
#include "pick/cubeeye_camera.hpp"
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

struct PickCandidate {
    int id = 0;
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
        int pointcloud_x = 0;
        int pointcloud_y = 0;
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
};

class PickProcessor final {
  public:
    explicit PickProcessor(PickProcessorConfig config);

    bool initialize();
    PickDetectionFrame process_detection_frame(const RgbdFrame& frame);
    PickViewerFrame process_viewer_frame(const RgbdFrame& frame) const;
    RgbCubeEyeOffset rgb_cubeeye_offset() const;
    PointCloudRoiConfig pointcloud_roi_config() const;
    RobotCalibrationConfig robot_calibration() const;
    bool update_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);
    bool update_pallet_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);
    bool update_rgb_cubeeye_offset(RgbCubeEyeOffset offset);
    bool update_pointcloud_roi_config(PointCloudRoiConfig config);
    bool update_robot_calibration(RobotCalibrationConfig config);

  private:
    struct RoiSnapshot {
        bool enabled = false;
        catcheye::roi::CameraRoiConfig config;
    };

    RoiSnapshot roi_snapshot() const;
    RoiSnapshot pallet_roi_snapshot() const;

    mutable std::mutex roi_mutex_;
    PickProcessorConfig config_;
    std::unique_ptr<catcheye::IDetector> detector_;
};

} // namespace catcheye::pick
