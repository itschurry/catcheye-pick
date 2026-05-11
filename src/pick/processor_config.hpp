#pragma once

#include <string>
#include <span>
#include <string_view>
#include <vector>

#include "CubeEyeFrame.h"

namespace catcheye::pick {

struct CubeEyeFrameSpec {
    std::string name;
    meere::sensor::FrameType type = meere::sensor::FrameType::Unknown;
};

struct PickProcessorConfig {
    bool detection_enabled = true;
    std::vector<CubeEyeFrameSpec> cubeeye_frames;
};

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value);
int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs);
std::string cubeeye_frame_label(meere::sensor::FrameType type);

} // namespace catcheye::pick
