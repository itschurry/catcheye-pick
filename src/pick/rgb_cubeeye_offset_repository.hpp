#pragma once

#include <string>

#include "pick/processor_config.hpp"

namespace catcheye::pick {

bool is_valid_rgb_cubeeye_offset(RgbCubeEyeOffset offset);
RgbCubeEyeOffset load_rgb_cubeeye_offset_config(const std::string& path);
std::string rgb_cubeeye_offset_to_json(RgbCubeEyeOffset offset);
bool save_rgb_cubeeye_offset_config(RgbCubeEyeOffset offset, const std::string& path);

} // namespace catcheye::pick
