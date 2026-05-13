#include "pick/processor_config.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace catcheye::pick {
namespace {

std::vector<std::string> split_csv(std::string_view value)
{
    std::vector<std::string> items;
    std::string current;
    for (const char ch : value) {
        if (ch == ',') {
            if (current.empty()) {
                throw std::invalid_argument("--cubeeye-frames contains an empty item");
            }
            items.push_back(current);
            current.clear();
            continue;
        }
        if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') {
            current.push_back(ch);
        }
    }
    if (current.empty()) {
        throw std::invalid_argument("--cubeeye-frames contains an empty item");
    }
    items.push_back(current);
    return items;
}

CubeEyeFrameSpec parse_cubeeye_frame_name(const std::string& name)
{
    if (name == "depth") {
        return {.name = "cubeeye_depth", .type = meere::sensor::FrameType::Depth};
    }
    if (name == "amplitude") {
        return {.name = "cubeeye_amplitude", .type = meere::sensor::FrameType::Amplitude};
    }
    if (name == "rgb") {
        return {.name = "cubeeye_rgb", .type = meere::sensor::FrameType::RGB};
    }
    if (name == "pointcloud") {
        return {.name = "cubeeye_pointcloud", .type = meere::sensor::FrameType::PointCloud};
    }
    throw std::invalid_argument("unsupported CubeEye frame: " + name);
}

} // namespace

std::vector<CubeEyeFrameSpec> parse_cubeeye_frames(std::string_view value)
{
    std::vector<CubeEyeFrameSpec> specs;
    bool has_depth = false;
    bool has_pointcloud = false;
    for (const std::string& item : split_csv(value)) {
        const CubeEyeFrameSpec spec = parse_cubeeye_frame_name(item);
        const auto duplicate =
            std::find_if(specs.begin(), specs.end(), [&](const CubeEyeFrameSpec& existing) { return existing.type == spec.type; });
        if (duplicate != specs.end()) {
            throw std::invalid_argument("duplicate CubeEye frame: " + item);
        }
        has_depth = has_depth || spec.type == meere::sensor::FrameType::Depth;
        has_pointcloud = has_pointcloud || spec.type == meere::sensor::FrameType::PointCloud;
        specs.push_back(spec);
    }
    if (specs.empty()) {
        throw std::invalid_argument("--cubeeye-frames requires at least one frame");
    }
    if (has_depth && has_pointcloud) {
        throw std::invalid_argument("CubeEye depth and pointcloud cannot be selected together");
    }
    return specs;
}

int cubeeye_frame_mask(std::span<const CubeEyeFrameSpec> specs)
{
    int mask = 0;
    for (const auto& spec : specs) {
        mask |= static_cast<int>(spec.type);
    }
    return mask;
}

std::string cubeeye_frame_label(meere::sensor::FrameType type)
{
    switch (type) {
    case meere::sensor::FrameType::Depth:
        return "depth";
    case meere::sensor::FrameType::Amplitude:
        return "amplitude";
    case meere::sensor::FrameType::RGB:
        return "rgb";
    case meere::sensor::FrameType::PointCloud:
        return "pointcloud";
    default:
        return "unknown";
    }
}

} // namespace catcheye::pick
