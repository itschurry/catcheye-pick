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
    float v = 0.0F;
    float tx_m = 0.0F;
    float ty_m = 0.0F;
    float tz_m = 0.0F;
    float roll_deg = 0.0F;
    float pitch_deg = 0.0F;
    float yaw_deg = 0.0F;
    int rgb_width = 2304;
    int rgb_height = 1296;
    float rgb_fx = 1220.0F;
    float rgb_fy = 1220.0F;
    float rgb_cx = 1152.0F;
    float rgb_cy = 648.0F;
    bool rgb_undistort_enabled = false;
    float rgb_dist_k1 = -0.28F;
    float rgb_dist_k2 = 0.08F;
    float rgb_dist_p1 = 0.0F;
    float rgb_dist_p2 = 0.0F;
    float rgb_dist_k3 = -0.01F;
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
    float min_confidence = 0.50F;
};

struct PickProcessorConfig {
    bool detection_enabled = true;
    catcheye::DetectorFactoryConfig detector;
    std::vector<CubeEyeFrameSpec> cubeeye_frames;
    int pointcloud_downsample = 4;
    int depth_projection_downsample = 4;
    std::string rgb_cubeeye_offset_config_path;
    RgbCubeEyeOffset rgb_cubeeye_offset;
    bool roi_enabled = false;
    std::string roi_config_path;
    catcheye::roi::CameraRoiConfig roi_config;
    bool pallet_roi_enabled = false;
    std::string pallet_roi_config_path;
    catcheye::roi::CameraRoiConfig pallet_roi_config;
    std::string pointcloud_roi_config_path;
    PointCloudRoiConfig pointcloud_roi_config;
    std::string robot_calibration_config_path;
    RobotCalibrationConfig robot_calibration;
};

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value);
int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs);
std::string cubeeye_frame_label(meere::sensor::FrameType type);

} // namespace catcheye::pick
