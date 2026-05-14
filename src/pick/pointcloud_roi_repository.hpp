#pragma once

#include <string>

#include "pick/processor_config.hpp"

namespace catcheye::pick {

bool is_valid_pointcloud_roi_config(const PointCloudRoiConfig& config);
PointCloudRoiConfig load_pointcloud_roi_config(const std::string& path);
std::string pointcloud_roi_config_to_json(const PointCloudRoiConfig& config);
bool save_pointcloud_roi_config(const PointCloudRoiConfig& config, const std::string& path);

} // namespace catcheye::pick
