#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "pick/processor.hpp"

namespace catcheye::pick {

std::string build_viewer_metadata(
    const PickViewerFrame& frame,
    bool viewer_only = true,
    const PickDetectionFrame* detection_frame = nullptr);
std::string build_detection_metadata(const PickDetectionFrame& frame);
std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame);

} // namespace catcheye::pick
