#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "catcheye/detection/detector.hpp"
#include "catcheye/input/frame.hpp"
#include "catcheye/roi/camera_roi_config.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor_config.hpp"

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
};

class PickProcessor final {
  public:
    explicit PickProcessor(PickProcessorConfig config);

    bool initialize();
    PickDetectionFrame process_detection_frame(
        const catcheye::input::Frame& camera_frame,
        const CubeEyeFrameSet& cubeeye_frames,
        std::uint64_t frame_index);
    PickViewerFrame process_viewer_frame(
        const std::optional<catcheye::input::Frame>& camera_frame,
        const CubeEyeFrameSet& cubeeye_frames,
        std::uint64_t frame_index) const;
    bool update_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);
    bool update_pallet_roi_config(const catcheye::roi::CameraRoiConfig& roi_config);

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

std::string build_viewer_metadata(
    const PickViewerFrame& frame,
    bool viewer_only = true,
    const PickDetectionFrame* detection_frame = nullptr);
std::string build_detection_metadata(const PickDetectionFrame& frame);
std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame);

} // namespace catcheye::pick
