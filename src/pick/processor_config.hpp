#pragma once

#include <string>

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
};

} // namespace catcheye::pick
