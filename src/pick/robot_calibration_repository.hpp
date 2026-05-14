#pragma once

#include <string>

#include "pick/processor_config.hpp"

namespace catcheye::pick {

bool is_valid_robot_calibration(const RobotCalibrationConfig& config);
RobotCalibrationConfig load_robot_calibration_config(const std::string& path);
std::string robot_calibration_to_json(const RobotCalibrationConfig& config);
bool save_robot_calibration_config(const RobotCalibrationConfig& config, const std::string& path);

} // namespace catcheye::pick
