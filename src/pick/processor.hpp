#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "catcheye/input/frame.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor_config.hpp"

namespace catcheye::pick {

struct ViewerPayload {
    std::string name;
    std::string kind;
    int width = 0;
    int height = 0;
    std::uint64_t source_timestamp_ms = 0;
    std::vector<std::uint8_t> jpeg;
};

struct PickViewerFrame {
    std::uint64_t frame_index = 0;
    std::vector<ViewerPayload> payloads;
};

class PickProcessor final {
  public:
    explicit PickProcessor(PickProcessorConfig config);

    bool initialize();
    PickViewerFrame process_viewer_frame(
        const std::optional<catcheye::input::Frame>& camera_frame,
        const CubeEyeFrameSet& cubeeye_frames,
        std::uint64_t frame_index) const;

  private:
    PickProcessorConfig config_;
};

std::string build_viewer_metadata(const PickViewerFrame& frame);
std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame);

} // namespace catcheye::pick
