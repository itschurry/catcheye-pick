#pragma once

#include <optional>
#include <string>
#include <vector>

#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/roi/camera_roi_config.hpp"

namespace catcheye::pick {

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

struct CameraIntrinsicsConfig {
    int width = 0;
    int height = 0;
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
};

struct DepthProjectionConfig {
    bool enabled = false;
    float min_depth_m = 0.05F;
    float max_depth_m = 0.0F;
};

struct ObjectPointConfig {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct ProductObjectConfig {
    std::string product_id;
    std::optional<ObjectPointConfig> pick_point_object_m;
};

struct PickProcessorConfig {
    bool detection_enabled = true;
    catcheye::DetectorFactoryConfig detector;
    bool roi_enabled = false;
    std::string roi_config_path;
    catcheye::roi::CameraRoiConfig roi_config;
    bool pallet_roi_enabled = false;
    std::string pallet_roi_config_path;
    catcheye::roi::CameraRoiConfig pallet_roi_config;
    std::string robot_calibration_config_path;
    RobotCalibrationConfig robot_calibration;
    CameraIntrinsicsConfig camera_intrinsics;
    DepthProjectionConfig depth_projection;
    std::vector<ProductObjectConfig> object_catalog;
};

} // namespace catcheye::pick
