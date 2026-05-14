#pragma once

#include <string>
#include <span>
#include <string_view>
#include <vector>

#include "CubeEyeFrame.h"
#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/roi/camera_roi_config.hpp"

namespace catcheye::pick {

struct CubeEyeFrameSpec {
    std::string name;
    meere::sensor::FrameType type = meere::sensor::FrameType::Unknown;
};

struct RgbCubeEyeOffset {
    float u = 0.0F;
    float v = 0.40F;
};

struct PalletCandidateConfig {
    bool enabled = true;
    float min_height_m = 0.005F;
    float max_height_m = 0.300F;
    int min_points = 80;
    int max_image_gap_px = 3;
};

struct PointCloudRoiConfig {
    bool enabled = false;
    bool apply_to_viewer = true;
    float min_x_m = -1.0F;
    float max_x_m = 1.0F;
    float min_y_m = -1.0F;
    float max_y_m = 1.0F;
    float min_z_m = 0.0F;
    float max_z_m = 2.0F;
};

struct RobotTransformConfig {
    float translation_m[3] = {0.0F, 0.0F, 0.0F};
    float rotation_rpy_deg[3] = {0.0F, 0.0F, 0.0F};
};

struct RobotCalibrationConfig {
    bool enabled = false;
    RobotTransformConfig r1;
    RobotTransformConfig r2;
};

struct PickProcessorConfig {
    bool detection_enabled = true;
    catcheye::DetectorFactoryConfig detector;
    std::vector<CubeEyeFrameSpec> cubeeye_frames;
    int pointcloud_downsample = 4;
    std::string rgb_cubeeye_offset_config_path;
    RgbCubeEyeOffset rgb_cubeeye_offset;
    bool roi_enabled = false;
    std::string roi_config_path;
    catcheye::roi::CameraRoiConfig roi_config;
    bool pallet_roi_enabled = false;
    std::string pallet_roi_config_path;
    catcheye::roi::CameraRoiConfig pallet_roi_config;
    std::string pallet_candidate_config_path;
    PalletCandidateConfig pallet_candidate_config;
    std::string pointcloud_roi_config_path;
    PointCloudRoiConfig pointcloud_roi_config;
    std::string robot_calibration_config_path;
    RobotCalibrationConfig robot_calibration;
};

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value);
int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs);
std::string cubeeye_frame_label(meere::sensor::FrameType type);

} // namespace catcheye::pick
