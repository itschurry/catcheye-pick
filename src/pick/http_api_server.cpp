#include "pick/http_api_server.hpp"

#include <cctype>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "catcheye/http/roi_api.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {
namespace {

struct JsonValue {
    enum class Type {
        Boolean,
        Integer,
    };

    Type type = Type::Integer;
    bool bool_value = false;
    int int_value = 0;
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
    return key == "framerate" || key == "auto_exposure" || key == "illumination" || key == "depth_range_min" || key == "depth_range_max";
}

bool is_bool_cubeeye_property(std::string_view key)
{
    return key == "auto_exposure" || key == "illumination";
}

bool valid_int_value(std::string_view key, int value)
{
    if (key == "framerate") {
        return value == 7 || value == 15 || value == 30;
    }
    if (key == "depth_range_min" || key == "depth_range_max") {
        return value >= 0 && value <= 8192;
    }
    return false;
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
        if (consumed != value_text.size()) {
            return false;
        }
        output.type = JsonValue::Type::Integer;
        output.int_value = value;
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

std::string rgb_cubeeye_offset_json(RgbCubeEyeOffset offset)
{
    std::ostringstream oss;
    oss << "{\"u\":" << offset.u << ",\"v\":" << offset.v << '}';
    return oss.str();
}

} // namespace

HttpApiServer::HttpApiServer(
    HttpApiServerConfig config,
    std::string roi_config_path,
    std::string pallet_roi_config_path,
    PickProcessor* processor,
    CubeEyeCameraSession* cubeeye)
    : config_(std::move(config)),
      roi_config_path_(std::move(roi_config_path)),
      pallet_roi_config_path_(std::move(pallet_roi_config_path)),
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
    return {200, "OK", rgb_cubeeye_offset_json(processor_->rgb_cubeeye_offset())};
}

catcheye::http::HttpResponse HttpApiServer::handle_put_rgb_cubeeye_offset(const std::string& body) const
{
    RgbCubeEyeOffset offset;
    if (!parse_float_field(body, "u", offset.u) || !parse_float_field(body, "v", offset.v)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB CubeEye offset JSON body")};
    }
    if (!processor_->update_rgb_cubeeye_offset(offset)) {
        return {400, "Bad Request", catcheye::http::json_error_body("RGB CubeEye offset out of range")};
    }
    return handle_get_rgb_cubeeye_offset();
}

} // namespace catcheye::pick
