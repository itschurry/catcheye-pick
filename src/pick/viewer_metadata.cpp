#include "pick/viewer_metadata.hpp"

#include <chrono>
#include <cstdint>
#include <sstream>

#include "catcheye/roi/roi_repository.hpp"

namespace catcheye::pick {
namespace {

std::uint64_t wall_clock_millis()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string escape_json(std::string_view value)
{
    std::ostringstream oss;
    for (const char ch : value) {
        switch (ch) {
        case '"':
            oss << "\\\"";
            break;
        case '\\':
            oss << "\\\\";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << ch;
            break;
        }
    }
    return oss.str();
}

void append_detection_fields(std::ostringstream& oss, const PickDetectionFrame& frame)
{
    oss << "\"detection_count\":" << frame.detections.size() << ",\"detections\":[";
    for (std::size_t i = 0; i < frame.detections.size(); ++i) {
        const auto& detection = frame.detections[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{\"class_id\":" << detection.class_id << ",\"class_name\":\"" << escape_json(detection.class_name) << "\""
            << ",\"score\":" << detection.score << ",\"box\":{\"x\":" << detection.box.x << ",\"y\":" << detection.box.y
            << ",\"width\":" << detection.box.width << ",\"height\":" << detection.box.height << "},\"position\":";
        if (detection.position.has_value()) {
            const auto& position = *detection.position;
            oss << "{\"x\":" << position.x << ",\"y\":" << position.y << ",\"z\":" << position.z
                << ",\"sample_count\":" << position.sample_count << ",\"pointcloud_x\":" << position.pointcloud_x
                << ",\"pointcloud_y\":" << position.pointcloud_y << ",\"bbox3d\":{\"min_x\":" << position.min_x
                << ",\"min_y\":" << position.min_y << ",\"min_z\":" << position.min_z << ",\"max_x\":" << position.max_x
                << ",\"max_y\":" << position.max_y << ",\"max_z\":" << position.max_z << "}}";
        } else {
            oss << "null";
        }
        oss << "}";
    }
    oss << "]";
}

void append_robot_point(std::ostringstream& oss, const RobotPoint& point)
{
    oss << "{\"x\":" << point.x << ",\"y\":" << point.y << ",\"z\":" << point.z << "}";
}

void append_pick_candidate_fields(std::ostringstream& oss, const PickDetectionFrame* frame)
{
    if (frame == nullptr) {
        oss << "\"pick_candidate_count\":0,\"pick_candidates\":[]";
        return;
    }

    oss << "\"pick_candidate_count\":" << frame->pick_candidates.size() << ",\"pick_candidates\":[";
    for (std::size_t i = 0; i < frame->pick_candidates.size(); ++i) {
        const auto& candidate = frame->pick_candidates[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{\"id\":" << candidate.id << ",\"product_id\":\"" << escape_json(candidate.product_id)
            << "\",\"confidence\":" << candidate.confidence << ",\"center_camera_m\":[" << candidate.center_x << ','
            << candidate.center_y << ',' << candidate.center_z << "],\"pose_camera\":{\"translation\":[" << candidate.center_x << ','
            << candidate.center_y << ',' << candidate.center_z << "],\"rotation_rpy_deg\":[" << candidate.roll_deg << ','
            << candidate.pitch_deg << ',' << candidate.yaw_deg << "]},\"bbox_camera_m\":[" << candidate.min_x << ','
            << candidate.min_y << ',' << candidate.min_z << ',' << candidate.max_x << ',' << candidate.max_y << ','
            << candidate.max_z << "],\"pick_point_camera_m\":[" << candidate.pick_x << ',' << candidate.pick_y << ','
            << candidate.pick_z << "],\"robot\":";
        if (candidate.r1.has_value() && candidate.r2.has_value()) {
            oss << "{\"r1\":";
            append_robot_point(oss, *candidate.r1);
            oss << ",\"r2\":";
            append_robot_point(oss, *candidate.r2);
            oss << "}";
        } else {
            oss << "null";
        }
        oss << "}";
    }
    oss << "]";
}

} // namespace

std::string build_viewer_metadata(const PickViewerFrame& frame, bool viewer_only, const PickDetectionFrame* detection_frame)
{
    std::ostringstream oss;
    oss << "{\"type\":\"viewer_frame\","
        << "\"viewer_only\":" << (viewer_only ? "true" : "false") << ',' << "\"frame_index\":" << frame.frame_index << ','
        << "\"wall_timestamp_ms\":" << wall_clock_millis() << ',' << "\"roi_enabled\":" << (frame.roi_enabled ? "true" : "false") << ','
        << "\"roi\":" << catcheye::roi::RoiRepository::to_json_string(frame.roi_config, 0) << ','
        << "\"pallet_roi_enabled\":" << (frame.pallet_roi_enabled ? "true" : "false") << ','
        << "\"pallet_roi\":" << catcheye::roi::RoiRepository::to_json_string(frame.pallet_roi_config, 0) << ',';
    if (detection_frame != nullptr) {
        append_detection_fields(oss, *detection_frame);
        oss << ',';
    }
    append_pick_candidate_fields(oss, detection_frame);
    oss << ',';
    oss << "\"streams\":[";
    for (std::size_t i = 0; i < frame.payloads.size(); ++i) {
        const auto& payload = frame.payloads[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{"
            << "\"name\":\"" << payload.name << "\","
            << "\"kind\":\"" << payload.kind << "\","
            << "\"encoding\":\"" << payload.encoding << "\","
            << "\"payload_index\":" << i << ',' << "\"width\":" << payload.width << ',' << "\"height\":" << payload.height << ','
            << "\"point_count\":" << payload.point_count << ',' << "\"stride\":" << payload.stride << ','
            << "\"source_timestamp_ms\":" << payload.source_timestamp_ms << ',' << "\"payload_size\":" << payload.bytes.size() << "}";
    }
    oss << "]}";
    return oss.str();
}

std::string build_detection_metadata(const PickDetectionFrame& frame)
{
    std::ostringstream oss;
    oss << "{\"viewer_only\":false,";
    append_detection_fields(oss, frame);
    oss << ',';
    append_pick_candidate_fields(oss, &frame);
    oss << "}";
    return oss.str();
}

std::vector<std::span<const std::uint8_t>> viewer_payload_spans(const PickViewerFrame& frame)
{
    std::vector<std::span<const std::uint8_t>> spans;
    spans.reserve(frame.payloads.size());
    for (const auto& payload : frame.payloads) {
        spans.emplace_back(payload.bytes.data(), payload.bytes.size());
    }
    return spans;
}

} // namespace catcheye::pick
