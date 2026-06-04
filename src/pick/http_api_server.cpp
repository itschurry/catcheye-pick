#include "pick/http_api_server.hpp"

#include <cctype>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "catcheye/http/roi_api.hpp"
#include "pick/config_json.hpp"
#include "pick/processor.hpp"
#include "pick/robot_calibration_repository.hpp"

namespace catcheye::pick {
namespace {

std::string trim(std::string value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
        value.erase(value.begin());
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
        value.pop_back();
    }
    return value;
}

bool parse_float_field(std::string_view body, std::string_view key, float& output)
{
    const std::string quoted_key = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = body.find(quoted_key);
    if (key_pos == std::string_view::npos) {
        return false;
    }
    const std::size_t colon_pos = body.find(':', key_pos + quoted_key.size());
    if (colon_pos == std::string_view::npos) {
        return false;
    }
    const std::size_t end_pos = body.find_first_of(",}", colon_pos + 1U);
    const std::string value_text = trim(std::string(body.substr(
        colon_pos + 1U,
        end_pos == std::string_view::npos ? std::string_view::npos : end_pos - colon_pos - 1U)));
    try {
        std::size_t consumed = 0;
        output = std::stof(value_text, &consumed);
        return consumed == value_text.size();
    } catch (...) {
        return false;
    }
}

std::string read_json_file(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open JSON config: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string body = buffer.str();
    if (body.empty()) {
        throw std::runtime_error("empty JSON config: " + path);
    }
    return body;
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

std::vector<std::string_view> json_object_blocks(std::string_view body, std::string_view array_key)
{
    const std::string quoted_key = "\"" + std::string(array_key) + "\"";
    const std::size_t key_pos = body.find(quoted_key);
    if (key_pos == std::string_view::npos) {
        return {};
    }
    const std::size_t array_open = body.find('[', key_pos + quoted_key.size());
    if (array_open == std::string_view::npos) {
        return {};
    }

    std::vector<std::string_view> blocks;
    int depth = 0;
    std::size_t object_start = std::string_view::npos;
    for (std::size_t i = array_open + 1U; i < body.size(); ++i) {
        const char ch = body[i];
        if (ch == '{') {
            if (depth == 0) {
                object_start = i;
            }
            ++depth;
        } else if (ch == '}') {
            --depth;
            if (depth == 0 && object_start != std::string_view::npos) {
                blocks.push_back(body.substr(object_start, i - object_start + 1U));
                object_start = std::string_view::npos;
            }
        } else if (ch == ']' && depth == 0) {
            break;
        }
    }
    return blocks;
}

bool parse_float_array3_field(std::string_view body, std::string_view key, RobotPoint& point)
{
    std::vector<float> values;
    if (!parse_json_float_array_field(body, key, values) || values.size() != 3U) {
        return false;
    }
    point = RobotPoint{
        .x = values[0],
        .y = values[1],
        .z = values[2],
    };
    return true;
}

bool parse_float_array4_field(std::string_view body, std::string_view key, float output[4])
{
    std::vector<float> values;
    if (!parse_json_float_array_field(body, key, values) || values.size() != 4U) {
        return false;
    }
    for (std::size_t i = 0; i < 4U; ++i) {
        output[i] = values[i];
    }
    return true;
}

bool parse_pose_estimates_body(std::string_view body, std::vector<PoseEstimate>& estimates)
{
    const auto blocks = json_object_blocks(body, "estimates");
    if (blocks.empty()) {
        return false;
    }

    std::vector<PoseEstimate> parsed;
    parsed.reserve(blocks.size());
    for (const std::string_view block : blocks) {
        PoseEstimate estimate;
        if (!parse_json_string_field(block, "object_id", estimate.object_id) ||
            !parse_json_string_field(block, "product_id", estimate.product_id) ||
            !parse_json_float_field(block, "confidence", estimate.confidence) ||
            !parse_float_array3_field(block, "translation_m", estimate.pose_camera.translation_m) ||
            !parse_float_array4_field(block, "rotation_quat_xyzw", estimate.pose_camera.rotation_quat_xyzw)) {
            return false;
        }
        parsed.push_back(std::move(estimate));
    }
    estimates = std::move(parsed);
    return true;
}

std::string pose_estimates_to_json(const std::vector<PoseEstimate>& estimates)
{
    std::ostringstream oss;
    oss << "{\"pose_estimate_count\":" << estimates.size() << ",\"pose_estimates\":[";
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        const auto& estimate = estimates[i];
        if (i > 0) {
            oss << ',';
        }
        oss << "{\"object_id\":\"" << escape_json(estimate.object_id) << "\",\"product_id\":\""
            << escape_json(estimate.product_id) << "\",\"confidence\":" << estimate.confidence
            << ",\"pose_camera\":{\"translation_m\":[" << estimate.pose_camera.translation_m.x << ','
            << estimate.pose_camera.translation_m.y << ',' << estimate.pose_camera.translation_m.z
            << "],\"rotation_quat_xyzw\":[" << estimate.pose_camera.rotation_quat_xyzw[0] << ','
            << estimate.pose_camera.rotation_quat_xyzw[1] << ',' << estimate.pose_camera.rotation_quat_xyzw[2] << ','
            << estimate.pose_camera.rotation_quat_xyzw[3] << "]},\"pick_point_camera_m\":["
            << estimate.pick_point_camera_m.x << ',' << estimate.pick_point_camera_m.y << ','
            << estimate.pick_point_camera_m.z << "],\"robot\":";
        if (estimate.r1.has_value() && estimate.r2.has_value()) {
            oss << "{\"r1\":{\"x\":" << estimate.r1->x << ",\"y\":" << estimate.r1->y << ",\"z\":" << estimate.r1->z
                << "},\"r2\":{\"x\":" << estimate.r2->x << ",\"y\":" << estimate.r2->y << ",\"z\":" << estimate.r2->z << "}}";
        } else {
            oss << "null";
        }
        oss << "}";
    }
    oss << "]}";
    return oss.str();
}

} // namespace

HttpApiServer::HttpApiServer(
    HttpApiServerConfig config,
    std::string roi_config_path,
    std::string pallet_roi_config_path,
    std::string intrinsics_config_path,
    std::string extrinsics_config_path,
    std::string robot_calibration_config_path,
    std::string object_catalog_config_path,
    PickProcessor* processor)
    : config_(std::move(config)),
      roi_config_path_(std::move(roi_config_path)),
      pallet_roi_config_path_(std::move(pallet_roi_config_path)),
      intrinsics_config_path_(std::move(intrinsics_config_path)),
      extrinsics_config_path_(std::move(extrinsics_config_path)),
      robot_calibration_config_path_(std::move(robot_calibration_config_path)),
      object_catalog_config_path_(std::move(object_catalog_config_path)),
      processor_(processor)
{}

HttpApiServer::~HttpApiServer()
{
    stop();
}

bool HttpApiServer::start()
{
    if (server_ != nullptr) {
        return true;
    }
    if (processor_ == nullptr || config_.port <= 0) {
        return false;
    }

    server_ = std::make_unique<catcheye::http::HttpServer>(catcheye::http::HttpServerConfig{
        .bind_address = config_.bind_address,
        .port = config_.port,
    });

    server_->add_route("/api/device-info", [](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return catcheye::http::HttpResponse{200, "OK", R"({"app":"catcheye-pick","kind":"pick"})"};
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    catcheye::http::register_roi_routes(
        *server_,
        catcheye::http::RoiApiConfig{
            .person_roi_path = roi_config_path_,
            .pallet_roi_path = pallet_roi_config_path_,
            .apply = [this](catcheye::http::RoiConfigKind kind, const catcheye::roi::CameraRoiConfig& roi_config) {
                return kind == catcheye::http::RoiConfigKind::Pallet
                    ? processor_->update_pallet_roi_config(roi_config)
                    : processor_->update_roi_config(roi_config);
            },
        });

    server_->add_route("/api/camera/intrinsics", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_intrinsics();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/camera/extrinsics", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_extrinsics();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/objects", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_objects();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/pose-estimates", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_pose_estimates();
        }
        if (request.method == "PUT") {
            return handle_put_pose_estimates(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/robot-calibration", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_robot_calibration();
        }
        if (request.method == "PUT") {
            return handle_put_robot_calibration(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    if (!server_->start()) {
        server_.reset();
        return false;
    }
    std::cerr << "HTTP API listening on " << config_.bind_address << ':' << config_.port << '\n';
    return true;
}

void HttpApiServer::stop()
{
    if (server_ != nullptr) {
        server_->stop();
        server_.reset();
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_get_intrinsics() const
{
    try {
        return {200, "OK", read_json_file(intrinsics_config_path_)};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_get_extrinsics() const
{
    try {
        return {200, "OK", read_json_file(extrinsics_config_path_)};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_get_objects() const
{
    try {
        return {200, "OK", read_json_file(object_catalog_config_path_)};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_get_pose_estimates() const
{
    return {200, "OK", pose_estimates_to_json(processor_->pose_estimates())};
}

catcheye::http::HttpResponse HttpApiServer::handle_put_pose_estimates(const std::string& body) const
{
    std::vector<PoseEstimate> estimates;
    if (!parse_pose_estimates_body(body, estimates)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid pose estimates JSON body")};
    }
    if (!processor_->update_pose_estimates(std::move(estimates))) {
        return {400, "Bad Request", catcheye::http::json_error_body("pose estimates failed validation")};
    }
    return handle_get_pose_estimates();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_robot_calibration() const
{
    try {
        return {200, "OK", robot_calibration_to_json(load_robot_calibration_config(robot_calibration_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_robot_calibration(const std::string& body) const
{
    RobotCalibrationConfig config;
    if (!parse_json_bool_field(body, "enabled", config.enabled)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid robot calibration JSON body")};
    }
    parse_float_field(body, "min_confidence", config.min_confidence);
    parse_float_field(body, "r1_tx_m", config.r1.translation_m[0]);
    parse_float_field(body, "r1_ty_m", config.r1.translation_m[1]);
    parse_float_field(body, "r1_tz_m", config.r1.translation_m[2]);
    parse_float_field(body, "r1_roll_deg", config.r1.rotation_rpy_deg[0]);
    parse_float_field(body, "r1_pitch_deg", config.r1.rotation_rpy_deg[1]);
    parse_float_field(body, "r1_yaw_deg", config.r1.rotation_rpy_deg[2]);
    parse_float_field(body, "r2_tx_m", config.r2.translation_m[0]);
    parse_float_field(body, "r2_ty_m", config.r2.translation_m[1]);
    parse_float_field(body, "r2_tz_m", config.r2.translation_m[2]);
    parse_float_field(body, "r2_roll_deg", config.r2.rotation_rpy_deg[0]);
    parse_float_field(body, "r2_pitch_deg", config.r2.rotation_rpy_deg[1]);
    parse_float_field(body, "r2_yaw_deg", config.r2.rotation_rpy_deg[2]);
    if (!is_valid_robot_calibration(config)) {
        return {400, "Bad Request", catcheye::http::json_error_body("robot calibration config out of range")};
    }
    if (!save_robot_calibration_config(config, robot_calibration_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save robot calibration config file")};
    }
    processor_->update_robot_calibration(config);
    return handle_get_robot_calibration();
}

} // namespace catcheye::pick
