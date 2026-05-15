#pragma once

#include <cstdint>
#include <optional>

#include "catcheye/input/frame.hpp"
#include "pick/cubeeye_camera.hpp"

namespace catcheye::pick {

struct RgbdFrame {
    std::uint64_t frame_index = 0;
    std::optional<catcheye::input::Frame> color;
    CubeEyeFrameSet depth;
};

} // namespace catcheye::pick
