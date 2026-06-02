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

} // namespace

HttpApiServer::HttpApiServer(
    HttpApiServerConfig config,
    std::string roi_config_path,
    std::string pallet_roi_config_path,
    std::string intrinsics_config_path,
    std::string extrinsics_config_path,
    std::string robot_calibration_config_path,
    PickProcessor* processor)
    : config_(std::move(config)),
      roi_config_path_(std::move(roi_config_path)),
      pallet_roi_config_path_(std::move(pallet_roi_config_path)),
      intrinsics_config_path_(std::move(intrinsics_config_path)),
      extrinsics_config_path_(std::move(extrinsics_config_path)),
      robot_calibration_config_path_(std::move(robot_calibration_config_path)),
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
