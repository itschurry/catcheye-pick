#include "pick/robot_calibration_repository.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "pick/config_json.hpp"

namespace catcheye::pick {
namespace {

bool finite_transform(const RobotTransformConfig& transform)
{
    for (float value : transform.translation_m) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    for (float value : transform.rotation_rpy_deg) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

void parse_transform(const std::string& json, std::string_view prefix, RobotTransformConfig& transform)
{
    parse_json_float_field(json, std::string(prefix) + "_tx_m", transform.translation_m[0]);
    parse_json_float_field(json, std::string(prefix) + "_ty_m", transform.translation_m[1]);
    parse_json_float_field(json, std::string(prefix) + "_tz_m", transform.translation_m[2]);
    parse_json_float_field(json, std::string(prefix) + "_roll_deg", transform.rotation_rpy_deg[0]);
    parse_json_float_field(json, std::string(prefix) + "_pitch_deg", transform.rotation_rpy_deg[1]);
    parse_json_float_field(json, std::string(prefix) + "_yaw_deg", transform.rotation_rpy_deg[2]);
}

void append_transform(std::ostringstream& oss, std::string_view prefix, const RobotTransformConfig& transform, bool trailing_comma)
{
    oss << "  \"" << prefix << "_tx_m\": " << transform.translation_m[0] << ",\n"
        << "  \"" << prefix << "_ty_m\": " << transform.translation_m[1] << ",\n"
        << "  \"" << prefix << "_tz_m\": " << transform.translation_m[2] << ",\n"
        << "  \"" << prefix << "_roll_deg\": " << transform.rotation_rpy_deg[0] << ",\n"
        << "  \"" << prefix << "_pitch_deg\": " << transform.rotation_rpy_deg[1] << ",\n"
        << "  \"" << prefix << "_yaw_deg\": " << transform.rotation_rpy_deg[2] << (trailing_comma ? ",\n" : "\n");
}

} // namespace

bool is_valid_robot_calibration(const RobotCalibrationConfig& config)
{
    return finite_transform(config.r1) && finite_transform(config.r2);
}

RobotCalibrationConfig load_robot_calibration_config(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to load robot calibration config: " + path);
    }

    const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    RobotCalibrationConfig config;
    if (!parse_json_bool_field(json, "enabled", config.enabled)) {
        throw std::runtime_error("invalid robot calibration config: " + path);
    }
    parse_transform(json, "r1", config.r1);
    parse_transform(json, "r2", config.r2);
    if (!is_valid_robot_calibration(config)) {
        throw std::runtime_error("invalid robot calibration config: " + path);
    }
    return config;
}

std::string robot_calibration_to_json(const RobotCalibrationConfig& config)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"enabled\": " << (config.enabled ? "true" : "false") << ",\n";
    append_transform(oss, "r1", config.r1, true);
    append_transform(oss, "r2", config.r2, false);
    oss << "}\n";
    return oss.str();
}

bool save_robot_calibration_config(const RobotCalibrationConfig& config, const std::string& path)
{
    if (!is_valid_robot_calibration(config)) {
        return false;
    }
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        return false;
    }
    output << robot_calibration_to_json(config);
    return static_cast<bool>(output);
}

} // namespace catcheye::pick
