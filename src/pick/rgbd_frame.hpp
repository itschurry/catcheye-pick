#pragma once

#include <cstdint>
#include <optional>

#include "catcheye/input/frame.hpp"

namespace catcheye::pick {

struct RgbdFrame {
    std::uint64_t frame_index = 0;
    std::optional<catcheye::input::Frame> color;
    std::optional<catcheye::input::Frame> depth_visual;
};

} // namespace catcheye::pick
