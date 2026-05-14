#include "pick/pointcloud_roi_repository.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "pick/config_json.hpp"

namespace catcheye::pick {

bool is_valid_pointcloud_roi_config(const PointCloudRoiConfig& config)
{
    return std::isfinite(config.min_x_m) && std::isfinite(config.max_x_m) && std::isfinite(config.min_y_m) &&
           std::isfinite(config.max_y_m) && std::isfinite(config.min_z_m) && std::isfinite(config.max_z_m) &&
           config.max_x_m > config.min_x_m && config.max_y_m > config.min_y_m && config.max_z_m > config.min_z_m;
}

PointCloudRoiConfig load_pointcloud_roi_config(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to load pointcloud ROI config: " + path);
    }

    const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    PointCloudRoiConfig config;
    if (!parse_json_bool_field(json, "enabled", config.enabled) || !parse_json_bool_field(json, "apply_to_viewer", config.apply_to_viewer) ||
        !parse_json_float_field(json, "min_x_m", config.min_x_m) ||
        !parse_json_float_field(json, "max_x_m", config.max_x_m) || !parse_json_float_field(json, "min_y_m", config.min_y_m) ||
        !parse_json_float_field(json, "max_y_m", config.max_y_m) || !parse_json_float_field(json, "min_z_m", config.min_z_m) ||
        !parse_json_float_field(json, "max_z_m", config.max_z_m) || !is_valid_pointcloud_roi_config(config)) {
        throw std::runtime_error("invalid pointcloud ROI config: " + path);
    }
    return config;
}

std::string pointcloud_roi_config_to_json(const PointCloudRoiConfig& config)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"enabled\": " << (config.enabled ? "true" : "false") << ",\n"
        << "  \"apply_to_viewer\": " << (config.apply_to_viewer ? "true" : "false") << ",\n"
        << "  \"min_x_m\": " << config.min_x_m << ",\n"
        << "  \"max_x_m\": " << config.max_x_m << ",\n"
        << "  \"min_y_m\": " << config.min_y_m << ",\n"
        << "  \"max_y_m\": " << config.max_y_m << ",\n"
        << "  \"min_z_m\": " << config.min_z_m << ",\n"
        << "  \"max_z_m\": " << config.max_z_m << "\n"
        << "}\n";
    return oss.str();
}

bool save_pointcloud_roi_config(const PointCloudRoiConfig& config, const std::string& path)
{
    if (!is_valid_pointcloud_roi_config(config)) {
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
    output << pointcloud_roi_config_to_json(config);
    return static_cast<bool>(output);
}

} // namespace catcheye::pick
