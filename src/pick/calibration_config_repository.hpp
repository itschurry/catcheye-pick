#pragma once

#include <string>

#include "pick/processor_config.hpp"

namespace catcheye::pick {

bool is_valid_rgb_intrinsic_config(RgbIntrinsicConfig config);
RgbIntrinsicConfig load_rgb_intrinsic_config(const std::string& path);
std::string rgb_intrinsic_config_to_json(RgbIntrinsicConfig config);
bool save_rgb_intrinsic_config(RgbIntrinsicConfig config, const std::string& path);

bool is_valid_rgb_cubeeye_extrinsic_config(RgbCubeEyeExtrinsicConfig config);
RgbCubeEyeExtrinsicConfig load_rgb_cubeeye_extrinsic_config(const std::string& path);
std::string rgb_cubeeye_extrinsic_config_to_json(RgbCubeEyeExtrinsicConfig config);
bool save_rgb_cubeeye_extrinsic_config(RgbCubeEyeExtrinsicConfig config, const std::string& path);

} // namespace catcheye::pick
