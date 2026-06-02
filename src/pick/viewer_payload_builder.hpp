#pragma once

#include "catcheye/input/frame.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {

ViewerPayload camera_payload(const catcheye::input::Frame& frame);
ViewerPayload depth_payload(const catcheye::input::Frame& frame);

} // namespace catcheye::pick
