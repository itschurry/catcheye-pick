#pragma once

#include <optional>

#include "catcheye/detection/bounding_box.hpp"
#include "catcheye/input/frame.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position(
    const catcheye::BoundingBox& box,
    const catcheye::input::Frame& camera_frame,
    const CubeEyeFrameEntry& pointcloud_entry);

} // namespace catcheye::pick
