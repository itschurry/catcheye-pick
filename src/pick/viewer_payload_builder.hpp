#pragma once

#include "catcheye/input/frame.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {

ViewerPayload camera_payload(const catcheye::input::Frame& frame);
ViewerPayload cubeeye_payload(const CubeEyeFrameEntry& entry, int pointcloud_downsample);

} // namespace catcheye::pick
