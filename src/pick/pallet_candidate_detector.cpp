#include "pick/pallet_candidate_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

#include "CubeEyePointCloudFrame.h"
#include "catcheye/roi/roi_geometry.hpp"

namespace catcheye::pick {
namespace {

struct PointSample {
    int image_x = 0;
    int image_y = 0;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct PlaneFit {
    float a = 0.0F;
    float b = 0.0F;
    float c = 0.0F;
};

bool valid_point(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0F;
}

bool solve3(float m[3][4], float out[3])
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::fabs(m[row][col]) > std::fabs(m[pivot][col])) {
                pivot = row;
            }
        }
        if (std::fabs(m[pivot][col]) < 1.0e-8F) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 4; ++k) {
                std::swap(m[col][k], m[pivot][k]);
            }
        }
        const float div = m[col][col];
        for (int k = col; k < 4; ++k) {
            m[col][k] /= div;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            const float factor = m[row][col];
            for (int k = col; k < 4; ++k) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }
    out[0] = m[0][3];
    out[1] = m[1][3];
    out[2] = m[2][3];
    return true;
}

std::optional<PlaneFit> fit_plane_z(const std::vector<PointSample>& samples)
{
    if (samples.size() < 3U) {
        return std::nullopt;
    }

    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    double sxz = 0.0;
    double syz = 0.0;
    for (const auto& p : samples) {
        sx += p.x;
        sy += p.y;
        sz += p.z;
        sxx += static_cast<double>(p.x) * p.x;
        syy += static_cast<double>(p.y) * p.y;
        sxy += static_cast<double>(p.x) * p.y;
        sxz += static_cast<double>(p.x) * p.z;
        syz += static_cast<double>(p.y) * p.z;
    }

    float matrix[3][4] = {
        {static_cast<float>(sxx), static_cast<float>(sxy), static_cast<float>(sx), static_cast<float>(sxz)},
        {static_cast<float>(sxy), static_cast<float>(syy), static_cast<float>(sy), static_cast<float>(syz)},
        {static_cast<float>(sx), static_cast<float>(sy), static_cast<float>(samples.size()), static_cast<float>(sz)},
    };
    float out[3] = {};
    if (!solve3(matrix, out)) {
        return std::nullopt;
    }
    return PlaneFit{.a = out[0], .b = out[1], .c = out[2]};
}

std::optional<PlaneFit> fit_plane_z_from_three(const PointSample& p0, const PointSample& p1, const PointSample& p2)
{
    float matrix[3][4] = {
        {p0.x, p0.y, 1.0F, p0.z},
        {p1.x, p1.y, 1.0F, p1.z},
        {p2.x, p2.y, 1.0F, p2.z},
    };
    float out[3] = {};
    if (!solve3(matrix, out)) {
        return std::nullopt;
    }
    return PlaneFit{.a = out[0], .b = out[1], .c = out[2]};
}

float plane_distance_z(const PointSample& point, const PlaneFit& plane)
{
    const float numerator = std::fabs(point.z - ((plane.a * point.x) + (plane.b * point.y) + plane.c));
    const float denominator = std::sqrt((plane.a * plane.a) + (plane.b * plane.b) + 1.0F);
    return numerator / denominator;
}

std::optional<PlaneFit> fit_dominant_plane_z(const std::vector<PointSample>& samples, std::vector<PointSample>& inliers)
{
    if (samples.size() < 3U) {
        return std::nullopt;
    }

    constexpr std::size_t max_ransac_samples = 2500U;
    constexpr int iterations = 96;
    constexpr float inlier_tolerance_m = 0.010F;

    const std::size_t stride = std::max<std::size_t>(1U, samples.size() / max_ransac_samples);
    std::vector<PointSample> model_samples;
    model_samples.reserve((samples.size() / stride) + 1U);
    for (std::size_t i = 0; i < samples.size(); i += stride) {
        model_samples.push_back(samples[i]);
    }
    if (model_samples.size() < 3U) {
        return std::nullopt;
    }

    std::optional<PlaneFit> best_plane;
    int best_count = 0;
    double best_error = std::numeric_limits<double>::max();
    for (int i = 0; i < iterations; ++i) {
        const auto base = static_cast<std::size_t>(i);
        const std::size_t i0 = (base * 37U) % model_samples.size();
        const std::size_t i1 = ((base * 73U) + 17U) % model_samples.size();
        const std::size_t i2 = ((base * 109U) + 31U) % model_samples.size();
        if (i0 == i1 || i0 == i2 || i1 == i2) {
            continue;
        }

        const auto plane = fit_plane_z_from_three(model_samples[i0], model_samples[i1], model_samples[i2]);
        if (!plane.has_value()) {
            continue;
        }

        int count = 0;
        double error = 0.0;
        for (const auto& point : model_samples) {
            const float distance = plane_distance_z(point, *plane);
            if (distance <= inlier_tolerance_m) {
                ++count;
                error += distance;
            }
        }
        if (count > best_count || (count == best_count && error < best_error)) {
            best_plane = plane;
            best_count = count;
            best_error = error;
        }
    }

    if (!best_plane.has_value()) {
        return std::nullopt;
    }

    inliers.clear();
    inliers.reserve(samples.size());
    for (const auto& point : samples) {
        if (plane_distance_z(point, *best_plane) <= inlier_tolerance_m) {
            inliers.push_back(point);
        }
    }
    return fit_plane_z(inliers);
}

PickViewerFrame::PalletPlane to_plane(const PlaneFit& fit, int sample_count)
{
    const float nx = -fit.a;
    const float ny = -fit.b;
    const float nz = 1.0F;
    const float norm = std::sqrt((nx * nx) + (ny * ny) + (nz * nz));
    return PickViewerFrame::PalletPlane{
        .normal_x = nx / norm,
        .normal_y = ny / norm,
        .normal_z = nz / norm,
        .d = -fit.c / norm,
        .sample_count = sample_count,
    };
}

float height_above_pallet(const PointSample& point, const PickViewerFrame::PalletPlane& plane)
{
    const float signed_distance = (plane.normal_x * point.x) + (plane.normal_y * point.y) + (plane.normal_z * point.z) + plane.d;
    return -signed_distance;
}

bool point_in_roi(int x, int y, int width, int height, const catcheye::roi::CameraRoiConfig& roi)
{
    if (roi.image_width <= 0 || roi.image_height <= 0) {
        return false;
    }

    const double sx = static_cast<double>(roi.image_width) / static_cast<double>(width);
    const double sy = static_cast<double>(roi.image_height) / static_cast<double>(height);
    const catcheye::roi::Point point{static_cast<double>(x) * sx, static_cast<double>(y) * sy};
    for (const auto& zone : roi.allowed_zones) {
        if (zone.enabled && catcheye::roi::point_in_polygon(point, zone)) {
            return true;
        }
    }
    return false;
}

bool point_in_pointcloud_roi(float x, float y, float z, const PointCloudRoiConfig& roi)
{
    if (!roi.enabled) {
        return true;
    }
    return x >= roi.min_x_m && x <= roi.max_x_m && y >= roi.min_y_m && y <= roi.max_y_m && z >= roi.min_z_m && z <= roi.max_z_m;
}

PickViewerFrame::RobotPoint transform_point(const RobotTransformConfig& transform, const PickViewerFrame::PalletCandidate& candidate)
{
    constexpr float pi = 3.14159265358979323846F;
    const float roll = transform.rotation_rpy_deg[0] * pi / 180.0F;
    const float pitch = transform.rotation_rpy_deg[1] * pi / 180.0F;
    const float yaw = transform.rotation_rpy_deg[2] * pi / 180.0F;
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float r00 = cy * cp;
    const float r01 = (cy * sp * sr) - (sy * cr);
    const float r02 = (cy * sp * cr) + (sy * sr);
    const float r10 = sy * cp;
    const float r11 = (sy * sp * sr) + (cy * cr);
    const float r12 = (sy * sp * cr) - (cy * sr);
    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    return PickViewerFrame::RobotPoint{
        .x = (r00 * candidate.center_x) + (r01 * candidate.center_y) + (r02 * candidate.center_z) + transform.translation_m[0],
        .y = (r10 * candidate.center_x) + (r11 * candidate.center_y) + (r12 * candidate.center_z) + transform.translation_m[1],
        .z = (r20 * candidate.center_x) + (r21 * candidate.center_y) + (r22 * candidate.center_z) + transform.translation_m[2],
    };
}

} // namespace

PalletCandidateDetectionResult detect_pallet_candidates(
    const CubeEyeFrameEntry& pointcloud_entry,
    const catcheye::roi::CameraRoiConfig& pallet_roi,
    const PointCloudRoiConfig& pointcloud_roi,
    const PalletCandidateConfig& candidate_config,
    const RobotCalibrationConfig& robot_calibration)
{
    PalletCandidateDetectionResult result;
    if (!candidate_config.enabled) {
        return result;
    }

    const auto pointcloud = meere::sensor::frame_cast_pcl32f(pointcloud_entry.frame);
    if (!pointcloud || !pointcloud->frameDataX() || !pointcloud->frameDataY() || !pointcloud->frameDataZ()) {
        return result;
    }

    const int width = pointcloud->frameWidth();
    const int height = pointcloud->frameHeight();
    if (width <= 0 || height <= 0) {
        return result;
    }
    const auto expected_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (pointcloud->frameDataX()->size() < expected_count || pointcloud->frameDataY()->size() < expected_count ||
        pointcloud->frameDataZ()->size() < expected_count) {
        return result;
    }

    const auto* xs = pointcloud->frameDataX()->data();
    const auto* ys = pointcloud->frameDataY()->data();
    const auto* zs = pointcloud->frameDataZ()->data();
    std::vector<PointSample> roi_points;
    roi_points.reserve(expected_count / 4U);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!point_in_roi(x, y, width, height, pallet_roi)) {
                continue;
            }
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            if (valid_point(xs[index], ys[index], zs[index]) && point_in_pointcloud_roi(xs[index], ys[index], zs[index], pointcloud_roi)) {
                roi_points.push_back(PointSample{.image_x = x, .image_y = y, .x = xs[index], .y = ys[index], .z = zs[index]});
            }
        }
    }
    if (roi_points.size() < static_cast<std::size_t>(candidate_config.min_points)) {
        return result;
    }

    std::vector<PointSample> plane_points;
    const auto plane_fit = fit_dominant_plane_z(roi_points, plane_points);
    if (plane_points.size() < static_cast<std::size_t>(candidate_config.min_points)) {
        return result;
    }

    if (!plane_fit.has_value()) {
        return result;
    }
    result.plane = to_plane(*plane_fit, static_cast<int>(plane_points.size()));

    std::vector<std::uint8_t> mask(expected_count, 0U);
    for (const auto& point : roi_points) {
        const float height_from_plane = height_above_pallet(point, *result.plane);
        if (height_from_plane >= candidate_config.min_height_m && height_from_plane <= candidate_config.max_height_m) {
            mask[static_cast<std::size_t>(point.image_y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(point.image_x)] = 1U;
        }
    }

    std::vector<std::uint8_t> visited(expected_count, 0U);
    const int gap = candidate_config.max_image_gap_px;
    int candidate_id = 1;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const auto start_index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            if (mask[start_index] == 0U || visited[start_index] != 0U) {
                continue;
            }

            std::queue<std::pair<int, int>> queue;
            queue.emplace(x, y);
            visited[start_index] = 1U;
            PickViewerFrame::PalletCandidate candidate;
            candidate.id = candidate_id++;
            candidate.min_x = candidate.min_y = candidate.min_z = std::numeric_limits<float>::max();
            candidate.max_x = candidate.max_y = candidate.max_z = -std::numeric_limits<float>::max();
            float sum_x = 0.0F;
            float sum_y = 0.0F;
            float sum_z = 0.0F;
            int min_ix = x;
            int min_iy = y;
            int max_ix = x;
            int max_iy = y;

            while (!queue.empty()) {
                const auto [cx, cy] = queue.front();
                queue.pop();
                const auto index = static_cast<std::size_t>(cy) * static_cast<std::size_t>(width) + static_cast<std::size_t>(cx);
                const PointSample point{.image_x = cx, .image_y = cy, .x = xs[index], .y = ys[index], .z = zs[index]};
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;
                candidate.sample_count++;
                candidate.height_m = std::max(candidate.height_m, height_above_pallet(point, *result.plane));
                candidate.min_x = std::min(candidate.min_x, point.x);
                candidate.min_y = std::min(candidate.min_y, point.y);
                candidate.min_z = std::min(candidate.min_z, point.z);
                candidate.max_x = std::max(candidate.max_x, point.x);
                candidate.max_y = std::max(candidate.max_y, point.y);
                candidate.max_z = std::max(candidate.max_z, point.z);
                min_ix = std::min(min_ix, cx);
                min_iy = std::min(min_iy, cy);
                max_ix = std::max(max_ix, cx);
                max_iy = std::max(max_iy, cy);

                for (int ny = std::max(0, cy - gap); ny <= std::min(height - 1, cy + gap); ++ny) {
                    for (int nx = std::max(0, cx - gap); nx <= std::min(width - 1, cx + gap); ++nx) {
                        const auto ni = static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) + static_cast<std::size_t>(nx);
                        if (mask[ni] != 0U && visited[ni] == 0U) {
                            visited[ni] = 1U;
                            queue.emplace(nx, ny);
                        }
                    }
                }
            }

            if (candidate.sample_count < candidate_config.min_points) {
                continue;
            }
            const float inv_count = 1.0F / static_cast<float>(candidate.sample_count);
            candidate.center_x = sum_x * inv_count;
            candidate.center_y = sum_y * inv_count;
            candidate.center_z = sum_z * inv_count;
            candidate.area_m2 = std::max(0.0F, candidate.max_x - candidate.min_x) * std::max(0.0F, candidate.max_y - candidate.min_y);
            const float point_score = std::min(1.0F, static_cast<float>(candidate.sample_count) / static_cast<float>(candidate_config.min_points * 4));
            const float height_score = std::min(1.0F, candidate.height_m / std::max(candidate_config.min_height_m * 4.0F, 0.001F));
            const float compact_score = std::min(1.0F, static_cast<float>(candidate.sample_count) /
                                                           static_cast<float>(std::max(1, (max_ix - min_ix + 1) * (max_iy - min_iy + 1))));
            candidate.confidence = std::clamp((point_score * 0.45F) + (height_score * 0.35F) + (compact_score * 0.20F), 0.0F, 1.0F);
            if (robot_calibration.enabled) {
                candidate.r1 = transform_point(robot_calibration.r1, candidate);
                candidate.r2 = transform_point(robot_calibration.r2, candidate);
            }
            result.candidates.push_back(candidate);
        }
    }

    std::sort(result.candidates.begin(), result.candidates.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.confidence > rhs.confidence;
    });
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        result.candidates[i].id = static_cast<int>(i + 1U);
    }
    return result;
}

} // namespace catcheye::pick
