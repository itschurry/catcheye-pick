#include "pick/http_api_server.hpp"

#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "catcheye/http/roi_api.hpp"
#include "pick/config_json.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/pallet_candidate_repository.hpp"
#include "pick/pointcloud_roi_repository.hpp"
#include "pick/processor.hpp"
#include "pick/rgb_cubeeye_offset_repository.hpp"
#include "pick/robot_calibration_repository.hpp"

namespace catcheye::pick {
namespace {

struct JsonValue {
    enum class Type {
        Boolean,
        Integer,
        Float,
    };

    Type type = Type::Integer;
    bool bool_value = false;
    int int_value = 0;
    float float_value = 0.0F;
};

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

bool is_supported_cubeeye_property(std::string_view key)
{
    return key == "framerate" || key == "auto_exposure" || key == "illumination" || key == "depth_range_min" || key == "depth_range_max" ||
           key == "amplitude_time_filter" || key == "depth_average_median_filter" || key == "depth_time_filter" ||
           key == "flying_pixel_remove_filter" || key == "noise_filter1" || key == "noise_filter2" || key == "noise_filter3" ||
           key == "amplitude_threshold_min" || key == "amplitude_threshold_max" || key == "amplitude_time_spatial_threshold" ||
           key == "amplitude_time_temporal_threshold" || key == "depth_average_median_max_n" || key == "depth_offset" ||
           key == "depth_time_spatial_threshold" || key == "depth_time_temporal_threshold" || key == "flying_pixel_remove_threshold" ||
           key == "integration_time" || key == "motion_blur_frequency" || key == "motion_blur_threshold" ||
           key == "motion_blur_threshold2" || key == "scattering_threshold";
}

bool is_bool_cubeeye_property(std::string_view key)
{
    return key == "auto_exposure" || key == "illumination" || key == "amplitude_time_filter" || key == "depth_average_median_filter" ||
           key == "depth_time_filter" || key == "flying_pixel_remove_filter" || key == "noise_filter1" || key == "noise_filter2" ||
           key == "noise_filter3";
}

bool is_float_cubeeye_property(std::string_view key)
{
    return key == "amplitude_time_spatial_threshold" || key == "amplitude_time_temporal_threshold" ||
           key == "depth_time_spatial_threshold" || key == "depth_time_temporal_threshold";
}

bool valid_int_value(std::string_view key, int value)
{
    if (key == "framerate") {
        return value == 7 || value == 15 || value == 30;
    }
    if (key == "depth_range_min" || key == "depth_range_max") {
        return value >= 0 && value <= 8192;
    }
    return value >= 0;
}

bool parse_value_body(std::string_view body, JsonValue& output)
{
    const std::size_t key_pos = body.find("\"value\"");
    if (key_pos == std::string_view::npos) {
        return false;
    }
    const std::size_t colon_pos = body.find(':', key_pos);
    if (colon_pos == std::string_view::npos) {
        return false;
    }

    std::string value_text = trim(std::string(body.substr(colon_pos + 1U)));
    if (!value_text.empty() && value_text.back() == '}') {
        value_text.pop_back();
    }
    value_text = trim(value_text);
    if (value_text == "true" || value_text == "false") {
        output.type = JsonValue::Type::Boolean;
        output.bool_value = value_text == "true";
        return true;
    }

    try {
        std::size_t consumed = 0;
        const int value = std::stoi(value_text, &consumed);
        if (consumed == value_text.size()) {
            output.type = JsonValue::Type::Integer;
            output.int_value = value;
            return true;
        }
    } catch (...) {
    }

    try {
        std::size_t consumed = 0;
        const float value = std::stof(value_text, &consumed);
        if (consumed != value_text.size() || !std::isfinite(value)) {
            return false;
        }
        output.type = JsonValue::Type::Float;
        output.float_value = value;
        return true;
    } catch (...) {
        return false;
    }
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

} // namespace

HttpApiServer::HttpApiServer(
    HttpApiServerConfig config,
    std::string roi_config_path,
    std::string pallet_roi_config_path,
    std::string rgb_cubeeye_offset_config_path,
    std::string pallet_candidate_config_path,
    std::string pointcloud_roi_config_path,
    std::string robot_calibration_config_path,
    PickProcessor* processor,
    CubeEyeCameraSession* cubeeye)
    : config_(std::move(config)),
      roi_config_path_(std::move(roi_config_path)),
      pallet_roi_config_path_(std::move(pallet_roi_config_path)),
      rgb_cubeeye_offset_config_path_(std::move(rgb_cubeeye_offset_config_path)),
      pallet_candidate_config_path_(std::move(pallet_candidate_config_path)),
      pointcloud_roi_config_path_(std::move(pointcloud_roi_config_path)),
      robot_calibration_config_path_(std::move(robot_calibration_config_path)),
      processor_(processor),
      cubeeye_(cubeeye)
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

    server_->add_route("/api/cubeeye/properties", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_cubeeye_properties();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    constexpr std::string_view property_prefix = "/api/cubeeye/properties/";
    constexpr std::size_t property_prefix_size = property_prefix.size();
    server_->add_prefix_route(std::string(property_prefix), [this, property_prefix_size](const catcheye::http::HttpRequest& request) {
        const std::string key = request.path.substr(property_prefix_size);
        if (request.method == "PUT") {
            return handle_put_cubeeye_property(key, request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-cubeeye-offset", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_rgb_cubeeye_offset();
        }
        if (request.method == "PUT") {
            return handle_put_rgb_cubeeye_offset(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/pallet-candidates/config", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_pallet_candidate_config();
        }
        if (request.method == "PUT") {
            return handle_put_pallet_candidate_config(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/pointcloud-roi", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_pointcloud_roi_config();
        }
        if (request.method == "PUT") {
            return handle_put_pointcloud_roi_config(request.body);
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

catcheye::http::HttpResponse HttpApiServer::handle_get_cubeeye_properties() const
{
    if (cubeeye_ == nullptr) {
        return {409, "Conflict", catcheye::http::json_error_body("CubeEye is not enabled")};
    }
    try {
        const auto properties = cubeeye_->properties_json();
        if (!properties.has_value()) {
            return {409, "Conflict", catcheye::http::json_error_body("CubeEye is not running")};
        }
        return {200, "OK", *properties};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_cubeeye_property(const std::string& key, const std::string& body) const
{
    if (cubeeye_ == nullptr) {
        return {409, "Conflict", catcheye::http::json_error_body("CubeEye is not enabled")};
    }
    if (!is_supported_cubeeye_property(key)) {
        return {400, "Bad Request", catcheye::http::json_error_body("unsupported CubeEye property")};
    }

    JsonValue value;
    if (!parse_value_body(body, value)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid property JSON body")};
    }

    bool updated = false;
    if (is_bool_cubeeye_property(key)) {
        if (value.type != JsonValue::Type::Boolean) {
            return {400, "Bad Request", catcheye::http::json_error_body("property value must be boolean")};
        }
        updated = cubeeye_->set_bool_property(key, value.bool_value);
    } else if (is_float_cubeeye_property(key)) {
        if (value.type != JsonValue::Type::Float && value.type != JsonValue::Type::Integer) {
            return {400, "Bad Request", catcheye::http::json_error_body("property value must be number")};
        }
        updated = cubeeye_->set_float_property(key, value.type == JsonValue::Type::Float ? value.float_value : static_cast<float>(value.int_value));
    } else {
        if (value.type != JsonValue::Type::Integer) {
            return {400, "Bad Request", catcheye::http::json_error_body("property value must be integer")};
        }
        if (!valid_int_value(key, value.int_value)) {
            return {400, "Bad Request", catcheye::http::json_error_body("property value out of range")};
        }
        updated = cubeeye_->set_int_property(key, value.int_value);
    }

    if (!updated) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to set CubeEye property")};
    }
    return handle_get_cubeeye_properties();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_rgb_cubeeye_offset() const
{
    try {
        return {200, "OK", rgb_cubeeye_offset_to_json(load_rgb_cubeeye_offset_config(rgb_cubeeye_offset_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_rgb_cubeeye_offset(const std::string& body) const
{
    RgbCubeEyeOffset offset;
    if (!parse_float_field(body, "u", offset.u) || !parse_float_field(body, "v", offset.v)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB CubeEye offset JSON body")};
    }
    if (!is_valid_rgb_cubeeye_offset(offset)) {
        return {400, "Bad Request", catcheye::http::json_error_body("RGB CubeEye offset out of range")};
    }
    if (!save_rgb_cubeeye_offset_config(offset, rgb_cubeeye_offset_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save RGB CubeEye offset config file")};
    }
    processor_->update_rgb_cubeeye_offset(offset);
    return handle_get_rgb_cubeeye_offset();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_pallet_candidate_config() const
{
    try {
        return {200, "OK", pallet_candidate_config_to_json(load_pallet_candidate_config(pallet_candidate_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_pallet_candidate_config(const std::string& body) const
{
    PalletCandidateConfig config;
    if (!parse_json_bool_field(body, "enabled", config.enabled) ||
        !parse_float_field(body, "min_height_m", config.min_height_m) ||
        !parse_float_field(body, "max_height_m", config.max_height_m) ||
        !parse_json_int_field(body, "min_points", config.min_points) ||
        !parse_json_int_field(body, "max_image_gap_px", config.max_image_gap_px)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid pallet candidate config JSON body")};
    }
    if (!is_valid_pallet_candidate_config(config)) {
        return {400, "Bad Request", catcheye::http::json_error_body("pallet candidate config out of range")};
    }
    if (!save_pallet_candidate_config(config, pallet_candidate_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save pallet candidate config file")};
    }
    processor_->update_pallet_candidate_config(config);
    return handle_get_pallet_candidate_config();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_pointcloud_roi_config() const
{
    try {
        return {200, "OK", pointcloud_roi_config_to_json(load_pointcloud_roi_config(pointcloud_roi_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_pointcloud_roi_config(const std::string& body) const
{
    PointCloudRoiConfig config;
    if (!parse_json_bool_field(body, "enabled", config.enabled) || !parse_json_bool_field(body, "apply_to_viewer", config.apply_to_viewer) ||
        !parse_float_field(body, "min_x_m", config.min_x_m) ||
        !parse_float_field(body, "max_x_m", config.max_x_m) || !parse_float_field(body, "min_y_m", config.min_y_m) ||
        !parse_float_field(body, "max_y_m", config.max_y_m) || !parse_float_field(body, "min_z_m", config.min_z_m) ||
        !parse_float_field(body, "max_z_m", config.max_z_m)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid pointcloud ROI JSON body")};
    }
    if (!is_valid_pointcloud_roi_config(config)) {
        return {400, "Bad Request", catcheye::http::json_error_body("pointcloud ROI out of range")};
    }
    if (!save_pointcloud_roi_config(config, pointcloud_roi_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save pointcloud ROI config file")};
    }
    processor_->update_pointcloud_roi_config(config);
    return handle_get_pointcloud_roi_config();
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
