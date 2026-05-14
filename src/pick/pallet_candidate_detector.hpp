#pragma once

#include <optional>
#include <vector>

#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {

struct PalletCandidateDetectionResult {
    std::optional<PickViewerFrame::PalletPlane> plane;
    std::vector<PickViewerFrame::PalletCandidate> candidates;
};

PalletCandidateDetectionResult detect_pallet_candidates(
    const CubeEyeFrameEntry& pointcloud_entry,
    const catcheye::roi::CameraRoiConfig& pallet_roi,
    const PointCloudRoiConfig& pointcloud_roi,
    const PalletCandidateConfig& candidate_config,
    const RobotCalibrationConfig& robot_calibration);

} // namespace catcheye::pick
