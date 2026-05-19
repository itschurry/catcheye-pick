#pragma once

#include <optional>

#include "catcheye/input/frame.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {

ViewerPayload camera_payload(const catcheye::input::Frame& frame, const RgbIntrinsicConfig& rgb_intrinsic);
ViewerPayload cubeeye_payload(const CubeEyeFrameEntry& entry, int pointcloud_downsample, const PointCloudRoiConfig& pointcloud_roi);
std::optional<ViewerPayload> projected_depth_payload(
    const catcheye::input::Frame& camera_frame,
    const CubeEyeFrameEntry& depth_entry,
    const std::optional<CubeEyeIntrinsics>& cubeeye_intrinsics,
    const RgbIntrinsicConfig& rgb_intrinsic,
    const RgbCubeEyeExtrinsicConfig& rgb_cubeeye_extrinsic,
    int stride);

} // namespace catcheye::pick
