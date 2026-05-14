#include "pick/object_position_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"

namespace catcheye::pick {
namespace {

struct PointSample {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

float percentile_value(std::vector<float>& values, float percentile)
{
    const auto index = static_cast<std::size_t>(std::clamp(percentile, 0.0F, 1.0F) * static_cast<float>(values.size() - 1U));
    const auto it = values.begin() + static_cast<std::ptrdiff_t>(index);
    std::nth_element(values.begin(), it, values.end());
    return *it;
}

struct CubeEyeSampleWindow {
    int min_x = 0;
    int max_x = 0;
    int min_y = 0;
    int max_y = 0;
    int center_x = 0;
    int center_y = 0;
};

std::optional<CubeEyeSampleWindow> map_detection_box(const catcheye::BoundingBox& box,
                                                     const catcheye::input::Frame& camera_frame,
                                                     int cubeeye_width,
                                                     int cubeeye_height,
                                                     RgbCubeEyeOffset rgb_cubeeye_offset)
{
    if (cubeeye_width <= 0 || cubeeye_height <= 0 || camera_frame.width <= 0 || camera_frame.height <= 0) {
        return std::nullopt;
    }

    const float center_x = box.x + (box.width * 0.5F);
    const float center_y = box.y + (box.height * 0.5F);
    const float cubeeye_u = std::clamp((center_x / static_cast<float>(camera_frame.width)) + rgb_cubeeye_offset.u, 0.0F, 1.0F);
    const float cubeeye_v = std::clamp((center_y / static_cast<float>(camera_frame.height)) + rgb_cubeeye_offset.v, 0.0F, 1.0F);
    const int cubeeye_x = std::clamp(static_cast<int>(cubeeye_u * static_cast<float>(cubeeye_width)), 0, cubeeye_width - 1);
    const int cubeeye_y = std::clamp(static_cast<int>(cubeeye_v * static_cast<float>(cubeeye_height)), 0, cubeeye_height - 1);

    const float left_u = std::clamp((box.x / static_cast<float>(camera_frame.width)) + rgb_cubeeye_offset.u, 0.0F, 1.0F);
    const float top_v = std::clamp((box.y / static_cast<float>(camera_frame.height)) + rgb_cubeeye_offset.v, 0.0F, 1.0F);
    const float right_u = std::clamp(((box.x + box.width) / static_cast<float>(camera_frame.width)) + rgb_cubeeye_offset.u, 0.0F, 1.0F);
    const float bottom_v =
        std::clamp(((box.y + box.height) / static_cast<float>(camera_frame.height)) + rgb_cubeeye_offset.v, 0.0F, 1.0F);

    return CubeEyeSampleWindow{
        .min_x = std::clamp(static_cast<int>(std::floor(std::min(left_u, right_u) * static_cast<float>(cubeeye_width))), 0, cubeeye_width - 1),
        .max_x = std::clamp(static_cast<int>(std::ceil(std::max(left_u, right_u) * static_cast<float>(cubeeye_width))), 0, cubeeye_width - 1),
        .min_y = std::clamp(static_cast<int>(std::floor(std::min(top_v, bottom_v) * static_cast<float>(cubeeye_height))), 0, cubeeye_height - 1),
        .max_y = std::clamp(static_cast<int>(std::ceil(std::max(top_v, bottom_v) * static_cast<float>(cubeeye_height))), 0, cubeeye_height - 1),
        .center_x = cubeeye_x,
        .center_y = cubeeye_y,
    };
}

std::optional<PickDetectionResult::ObjectPosition> summarize_samples(std::vector<PointSample>& samples,
                                                                      std::vector<float>& sample_zs,
                                                                      const CubeEyeSampleWindow& sample_window)
{
    if (sample_zs.empty()) {
        return std::nullopt;
    }

    const float min_depth = percentile_value(sample_zs, 0.10F);
    const float max_depth = percentile_value(sample_zs, 0.90F);
    float sum_x = 0.0F;
    float sum_y = 0.0F;
    float sum_z = 0.0F;
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    int filtered_count = 0;
    for (const auto& sample : samples) {
        if (sample.z < min_depth || sample.z > max_depth) {
            continue;
        }
        sum_x += sample.x;
        sum_y += sample.y;
        sum_z += sample.z;
        min_x = std::min(min_x, sample.x);
        min_y = std::min(min_y, sample.y);
        min_z = std::min(min_z, sample.z);
        max_x = std::max(max_x, sample.x);
        max_y = std::max(max_y, sample.y);
        max_z = std::max(max_z, sample.z);
        ++filtered_count;
    }

    if (filtered_count == 0) {
        return std::nullopt;
    }

    const float inv_count = 1.0F / static_cast<float>(filtered_count);
    return PickDetectionResult::ObjectPosition{
        .x = sum_x * inv_count,
        .y = sum_y * inv_count,
        .z = sum_z * inv_count,
        .sample_count = filtered_count,
        .pointcloud_x = sample_window.center_x,
        .pointcloud_y = sample_window.center_y,
        .min_x = min_x,
        .min_y = min_y,
        .min_z = min_z,
        .max_x = max_x,
        .max_y = max_y,
        .max_z = max_z,
    };
}

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position_from_pointcloud(const catcheye::BoundingBox& box,
                                                                                            const catcheye::input::Frame& camera_frame,
                                                                                            const CubeEyeFrameEntry& pointcloud_entry,
                                                                                            RgbCubeEyeOffset rgb_cubeeye_offset)
{
    const auto pointcloud = meere::sensor::frame_cast_pcl32f(pointcloud_entry.frame);
    if (!pointcloud || !pointcloud->frameDataX() || !pointcloud->frameDataY() || !pointcloud->frameDataZ()) {
        return std::nullopt;
    }

    const int width = pointcloud->frameWidth();
    const int height = pointcloud->frameHeight();
    const auto sample_window = map_detection_box(box, camera_frame, width, height, rgb_cubeeye_offset);
    if (!sample_window.has_value()) {
        return std::nullopt;
    }

    const auto expected_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const auto* xs = pointcloud->frameDataX();
    const auto* ys = pointcloud->frameDataY();
    const auto* zs = pointcloud->frameDataZ();
    if (xs->size() < expected_count || ys->size() < expected_count || zs->size() < expected_count) {
        return std::nullopt;
    }

    std::vector<PointSample> samples;
    std::vector<float> sample_zs;
    samples.reserve(static_cast<std::size_t>(std::max(0, sample_window->max_x - sample_window->min_x + 1)) *
                    static_cast<std::size_t>(std::max(0, sample_window->max_y - sample_window->min_y + 1)));

    const auto* x_data = xs->data();
    const auto* y_data = ys->data();
    const auto* z_data = zs->data();
    for (int y = sample_window->min_y; y <= sample_window->max_y; ++y) {
        for (int x = sample_window->min_x; x <= sample_window->max_x; ++x) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            const float px = x_data[index];
            const float py = y_data[index];
            const float pz = z_data[index];
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) || pz <= 0.0F) {
                continue;
            }
            samples.push_back(PointSample{.x = px, .y = py, .z = pz});
            sample_zs.push_back(pz);
        }
    }

    return summarize_samples(samples, sample_zs, *sample_window);
}

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position_from_depth(const catcheye::BoundingBox& box,
                                                                                       const catcheye::input::Frame& camera_frame,
                                                                                       const CubeEyeFrameEntry& depth_entry,
                                                                                       const CubeEyeIntrinsics& intrinsics,
                                                                                       RgbCubeEyeOffset rgb_cubeeye_offset)
{
    if (intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F) {
        return std::nullopt;
    }

    const auto depth = meere::sensor::frame_cast_basic16u(depth_entry.frame);
    if (!depth || !depth->frameData() || depth->frameData()->empty()) {
        return std::nullopt;
    }

    const int width = depth->frameWidth();
    const int height = depth->frameHeight();
    const auto sample_window = map_detection_box(box, camera_frame, width, height, rgb_cubeeye_offset);
    if (!sample_window.has_value()) {
        return std::nullopt;
    }

    const auto expected_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const auto* depth_values = depth->frameData();
    if (depth_values->size() < expected_count) {
        return std::nullopt;
    }

    std::vector<PointSample> samples;
    std::vector<float> sample_zs;
    samples.reserve(static_cast<std::size_t>(std::max(0, sample_window->max_x - sample_window->min_x + 1)) *
                    static_cast<std::size_t>(std::max(0, sample_window->max_y - sample_window->min_y + 1)));

    const auto* data = depth_values->data();
    for (int y = sample_window->min_y; y <= sample_window->max_y; ++y) {
        for (int x = sample_window->min_x; x <= sample_window->max_x; ++x) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            const float z = static_cast<float>(data[index]);
            if (!std::isfinite(z) || z <= 0.0F) {
                continue;
            }
            const float px = ((static_cast<float>(x) - intrinsics.cx) / intrinsics.fx) * z;
            const float py = ((static_cast<float>(y) - intrinsics.cy) / intrinsics.fy) * z;
            samples.push_back(PointSample{.x = px, .y = py, .z = z});
            sample_zs.push_back(z);
        }
    }

    return summarize_samples(samples, sample_zs, *sample_window);
}

} // namespace

std::optional<PickDetectionResult::ObjectPosition> estimate_object_position(const catcheye::BoundingBox& box,
                                                                            const catcheye::input::Frame& camera_frame,
                                                                            const CubeEyeFrameEntry& cubeeye_entry,
                                                                            const std::optional<CubeEyeIntrinsics>& cubeeye_intrinsics,
                                                                            RgbCubeEyeOffset rgb_cubeeye_offset)
{
    if (cubeeye_entry.spec.type == meere::sensor::FrameType::PointCloud) {
        return estimate_object_position_from_pointcloud(box, camera_frame, cubeeye_entry, rgb_cubeeye_offset);
    }
    if (cubeeye_entry.spec.type == meere::sensor::FrameType::Depth && cubeeye_intrinsics.has_value()) {
        return estimate_object_position_from_depth(box, camera_frame, cubeeye_entry, *cubeeye_intrinsics, rgb_cubeeye_offset);
    }
    return std::nullopt;
}

} // namespace catcheye::pick
