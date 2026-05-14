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
    float v = 0.40F;
};

struct PickProcessorConfig {
    bool detection_enabled = true;
    catcheye::DetectorFactoryConfig detector;
    std::vector<CubeEyeFrameSpec> cubeeye_frames;
    int pointcloud_downsample = 4;
    std::string rgb_cubeeye_offset_config_path;
    RgbCubeEyeOffset rgb_cubeeye_offset;
    bool roi_enabled = false;
    std::string roi_config_path;
    catcheye::roi::CameraRoiConfig roi_config;
    bool pallet_roi_enabled = false;
    std::string pallet_roi_config_path;
    catcheye::roi::CameraRoiConfig pallet_roi_config;
};

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value);
int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs);
std::string cubeeye_frame_label(meere::sensor::FrameType type);

} // namespace catcheye::pick
