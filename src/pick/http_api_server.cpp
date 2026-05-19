#include "pick/http_api_server.hpp"

#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "catcheye/http/roi_api.hpp"
#include "catcheye/input/rgb_intrinsic_calibrator.hpp"
#include "pick/config_json.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/pointcloud_roi_repository.hpp"
#include "pick/processor.hpp"
#include "pick/calibration_config_repository.hpp"
#include "pick/robot_calibration_repository.hpp"

namespace catcheye::pick {
namespace {

struct JsonValue {
    enum class Type {
        Boolean,
        Integer,
        Float,
        String,
    };

    Type type = Type::Integer;
    bool bool_value = false;
    int int_value = 0;
    float float_value = 0.0F;
    std::string string_value;
};

enum class RuntimePropertyType {
    Boolean,
    Integer,
    Float,
    Enum,
};

struct RuntimePropertySpec {
    std::string_view key;
    RuntimePropertyType type;
};

constexpr RuntimePropertySpec RGB_CAMERA_PROPERTIES[] = {
    {"ae-enable", RuntimePropertyType::Boolean},
    {"ae-metering-mode", RuntimePropertyType::Enum},
    {"ae-flicker-period", RuntimePropertyType::Integer},
    {"exposure-time-mode", RuntimePropertyType::Enum},
    {"exposure-time", RuntimePropertyType::Integer},
    {"exposure-value", RuntimePropertyType::Float},
    {"analogue-gain-mode", RuntimePropertyType::Enum},
    {"analogue-gain", RuntimePropertyType::Float},
    {"awb-enable", RuntimePropertyType::Boolean},
    {"awb-mode", RuntimePropertyType::Enum},
    {"af-mode", RuntimePropertyType::Enum},
    {"lens-position", RuntimePropertyType::Float},
    {"brightness", RuntimePropertyType::Float},
    {"contrast", RuntimePropertyType::Float},
    {"saturation", RuntimePropertyType::Float},
    {"sharpness", RuntimePropertyType::Float},
    {"gamma", RuntimePropertyType::Float},
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

std::optional<RuntimePropertySpec> find_rgb_camera_property(std::string_view key)
{
    for (const auto& spec : RGB_CAMERA_PROPERTIES) {
        if (spec.key == key) {
            return spec;
        }
    }
    return std::nullopt;
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
    if (value_text.size() >= 2U && value_text.front() == '"' && value_text.back() == '"') {
        output.type = JsonValue::Type::String;
        output.string_value = value_text.substr(1U, value_text.size() - 2U);
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

bool parse_bool_field(std::string_view body, std::string_view key, bool& output)
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
    if (value_text == "true") {
        output = true;
        return true;
    }
    if (value_text == "false") {
        output = false;
        return true;
    }
    return false;
}

bool parse_int_field(std::string_view body, std::string_view key, int& output)
{
    float value = 0.0F;
    if (!parse_float_field(body, key, value)) {
        return false;
    }
    const int integer = static_cast<int>(value);
    if (static_cast<float>(integer) != value) {
        return false;
    }
    output = integer;
    return true;
}

std::optional<catcheye::input::RgbIntrinsicCalibrationBoard> parse_rgb_intrinsic_board(std::string_view body)
{
    catcheye::input::RgbIntrinsicCalibrationBoard board;
    if (!parse_int_field(body, "pattern_width", board.pattern_width) ||
        !parse_int_field(body, "pattern_height", board.pattern_height) ||
        !parse_float_field(body, "square_size_m", board.square_size_m)) {
        return std::nullopt;
    }
    if (board.pattern_width <= 0 || board.pattern_height <= 0 || board.square_size_m <= 0.0F) {
        return std::nullopt;
    }
    return board;
}

} // namespace

HttpApiServer::HttpApiServer(
    HttpApiServerConfig config,
    std::string roi_config_path,
    std::string pallet_roi_config_path,
    std::string rgb_intrinsic_config_path,
    std::string rgb_cubeeye_extrinsic_config_path,
    std::string pointcloud_roi_config_path,
    std::string robot_calibration_config_path,
    PickProcessor* processor,
    catcheye::input::FrameSource* camera_source,
    CubeEyeCameraSession* cubeeye)
    : config_(std::move(config)),
      roi_config_path_(std::move(roi_config_path)),
      pallet_roi_config_path_(std::move(pallet_roi_config_path)),
      rgb_intrinsic_config_path_(std::move(rgb_intrinsic_config_path)),
      rgb_cubeeye_extrinsic_config_path_(std::move(rgb_cubeeye_extrinsic_config_path)),
      pointcloud_roi_config_path_(std::move(pointcloud_roi_config_path)),
      robot_calibration_config_path_(std::move(robot_calibration_config_path)),
      processor_(processor),
      camera_source_(camera_source),
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

    server_->add_route("/api/cubeeye/properties", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_cubeeye_properties();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-camera/properties", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_rgb_camera_properties();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-camera/intrinsic", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_rgb_intrinsic();
        }
        if (request.method == "PUT") {
            return handle_put_rgb_intrinsic(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-camera/intrinsic-calibration", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_rgb_intrinsic_calibration();
        }
        if (request.method == "DELETE") {
            return handle_delete_rgb_intrinsic_calibration();
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-camera/intrinsic-calibration/capture", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "POST") {
            return handle_post_rgb_intrinsic_capture(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    server_->add_route("/api/rgb-camera/intrinsic-calibration/solve", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "POST") {
            return handle_post_rgb_intrinsic_solve(request.body);
        }
        return catcheye::http::HttpResponse{405, "Method Not Allowed", catcheye::http::json_error_body("method not allowed")};
    });

    constexpr std::string_view rgb_camera_property_prefix = "/api/rgb-camera/properties/";
    constexpr std::size_t rgb_camera_property_prefix_size = rgb_camera_property_prefix.size();
    server_->add_prefix_route(std::string(rgb_camera_property_prefix), [this, rgb_camera_property_prefix_size](const catcheye::http::HttpRequest& request) {
        const std::string key = request.path.substr(rgb_camera_property_prefix_size);
        if (request.method == "PUT") {
            return handle_put_rgb_camera_property(key, request.body);
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

    server_->add_route("/api/rgb-cubeeye/extrinsic", [this](const catcheye::http::HttpRequest& request) {
        if (request.method == "GET") {
            return handle_get_rgb_cubeeye_extrinsic();
        }
        if (request.method == "PUT") {
            return handle_put_rgb_cubeeye_extrinsic(request.body);
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

catcheye::http::HttpResponse HttpApiServer::handle_get_rgb_camera_properties() const
{
    if (camera_source_ == nullptr) {
        return {409, "Conflict", catcheye::http::json_error_body("RGB camera is not enabled")};
    }

    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (const auto& spec : RGB_CAMERA_PROPERTIES) {
        const auto value = camera_source_->property_json(spec.key);
        if (!value.has_value()) {
            continue;
        }
        if (!first) {
            oss << ',';
        }
        first = false;
        oss << '"' << spec.key << "\":" << *value;
    }
    oss << "}";
    return {200, "OK", oss.str()};
}

catcheye::http::HttpResponse HttpApiServer::handle_put_rgb_camera_property(const std::string& key, const std::string& body) const
{
    if (camera_source_ == nullptr) {
        return {409, "Conflict", catcheye::http::json_error_body("RGB camera is not enabled")};
    }
    const auto spec = find_rgb_camera_property(key);
    if (!spec.has_value()) {
        return {400, "Bad Request", catcheye::http::json_error_body("unsupported RGB camera property")};
    }

    JsonValue value;
    if (!parse_value_body(body, value)) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid property JSON body")};
    }

    bool updated = false;
    switch (spec->type) {
        case RuntimePropertyType::Boolean:
            if (value.type != JsonValue::Type::Boolean) {
                return {400, "Bad Request", catcheye::http::json_error_body("property value must be boolean")};
            }
            updated = camera_source_->set_bool_property(key, value.bool_value);
            break;
        case RuntimePropertyType::Integer:
            if (value.type != JsonValue::Type::Integer) {
                return {400, "Bad Request", catcheye::http::json_error_body("property value must be integer")};
            }
            updated = camera_source_->set_int_property(key, value.int_value);
            break;
        case RuntimePropertyType::Float:
            if (value.type != JsonValue::Type::Float && value.type != JsonValue::Type::Integer) {
                return {400, "Bad Request", catcheye::http::json_error_body("property value must be number")};
            }
            updated = camera_source_->set_float_property(
                key,
                value.type == JsonValue::Type::Float ? value.float_value : static_cast<float>(value.int_value));
            break;
        case RuntimePropertyType::Enum:
            if (value.type != JsonValue::Type::String) {
                return {400, "Bad Request", catcheye::http::json_error_body("property value must be string")};
            }
            updated = camera_source_->set_string_property(key, value.string_value);
            break;
    }

    if (!updated) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to set RGB camera property")};
    }
    return handle_get_rgb_camera_properties();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_rgb_intrinsic() const
{
    try {
        return {200, "OK", rgb_intrinsic_config_to_json(load_rgb_intrinsic_config(rgb_intrinsic_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_rgb_intrinsic(const std::string& body) const
{
    RgbIntrinsicConfig config;
    try {
        config = load_rgb_intrinsic_config(rgb_intrinsic_config_path_);
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }

    bool has_field = false;
    float width = static_cast<float>(config.width);
    float height = static_cast<float>(config.height);
    if (parse_float_field(body, "width", width)) {
        config.width = static_cast<int>(width);
        has_field = true;
    }
    if (parse_float_field(body, "height", height)) {
        config.height = static_cast<int>(height);
        has_field = true;
    }
    has_field = parse_float_field(body, "fx", config.fx) || has_field;
    has_field = parse_float_field(body, "fy", config.fy) || has_field;
    has_field = parse_float_field(body, "cx", config.cx) || has_field;
    has_field = parse_float_field(body, "cy", config.cy) || has_field;
    has_field = parse_bool_field(body, "undistort_enabled", config.undistort_enabled) || has_field;
    has_field = parse_float_field(body, "dist_k1", config.dist_k1) || has_field;
    has_field = parse_float_field(body, "dist_k2", config.dist_k2) || has_field;
    has_field = parse_float_field(body, "dist_p1", config.dist_p1) || has_field;
    has_field = parse_float_field(body, "dist_p2", config.dist_p2) || has_field;
    has_field = parse_float_field(body, "dist_k3", config.dist_k3) || has_field;
    if (!has_field) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB intrinsic JSON body")};
    }
    if (!is_valid_rgb_intrinsic_config(config)) {
        return {400, "Bad Request", catcheye::http::json_error_body("RGB intrinsic out of range")};
    }
    if (!save_rgb_intrinsic_config(config, rgb_intrinsic_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save RGB intrinsic config file")};
    }
    processor_->update_rgb_intrinsic(config);
    return handle_get_rgb_intrinsic();
}

catcheye::http::HttpResponse HttpApiServer::handle_get_rgb_intrinsic_calibration() const
{
    try {
        const RgbIntrinsicConfig config = load_rgb_intrinsic_config(rgb_intrinsic_config_path_);
        std::lock_guard<std::mutex> lock(rgb_intrinsic_mutex_);
        const int capture_count = rgb_intrinsic_calibrator_ ? rgb_intrinsic_calibrator_->capture_count() : 0;
        std::ostringstream oss;
        oss << "{"
            << "\"capture_count\":" << capture_count << ','
            << "\"width\":" << config.width << ','
            << "\"height\":" << config.height << ','
            << "\"fx\":" << config.fx << ','
            << "\"fy\":" << config.fy << ','
            << "\"cx\":" << config.cx << ','
            << "\"cy\":" << config.cy << ','
            << "\"undistort_enabled\":" << (config.undistort_enabled ? "true" : "false") << ','
            << "\"dist_k1\":" << config.dist_k1 << ','
            << "\"dist_k2\":" << config.dist_k2 << ','
            << "\"dist_p1\":" << config.dist_p1 << ','
            << "\"dist_p2\":" << config.dist_p2 << ','
            << "\"dist_k3\":" << config.dist_k3
            << "}";
        return {200, "OK", oss.str()};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_delete_rgb_intrinsic_calibration() const
{
    std::lock_guard<std::mutex> lock(rgb_intrinsic_mutex_);
    rgb_intrinsic_calibrator_.reset();
    return {200, "OK", "{\"capture_count\":0}"};
}

catcheye::http::HttpResponse HttpApiServer::handle_post_rgb_intrinsic_capture(const std::string& body) const
{
    if (processor_ == nullptr) {
        return {409, "Conflict", catcheye::http::json_error_body("processor is not enabled")};
    }
    const auto board = parse_rgb_intrinsic_board(body);
    if (!board.has_value()) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB intrinsic board JSON body")};
    }
    const auto frame = processor_->latest_rgb_frame();
    if (!frame.has_value()) {
        return {409, "Conflict", catcheye::http::json_error_body("RGB frame is not available")};
    }

    try {
        std::lock_guard<std::mutex> lock(rgb_intrinsic_mutex_);
        if (!rgb_intrinsic_calibrator_) {
            rgb_intrinsic_calibrator_ = std::make_unique<catcheye::input::RgbIntrinsicCalibrator>(*board);
        }
        const bool captured = rgb_intrinsic_calibrator_->add_frame(*frame);
        std::ostringstream oss;
        oss << "{"
            << "\"captured\":" << (captured ? "true" : "false") << ','
            << "\"capture_count\":" << rgb_intrinsic_calibrator_->capture_count()
            << "}";
        return {200, "OK", oss.str()};
    } catch (const std::exception& e) {
        return {400, "Bad Request", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_post_rgb_intrinsic_solve(const std::string& body) const
{
    const auto board = parse_rgb_intrinsic_board(body);
    if (!board.has_value()) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB intrinsic board JSON body")};
    }

    try {
        std::lock_guard<std::mutex> lock(rgb_intrinsic_mutex_);
        if (!rgb_intrinsic_calibrator_) {
            return {409, "Conflict", catcheye::http::json_error_body("RGB intrinsic captures are empty")};
        }
        const auto result = rgb_intrinsic_calibrator_->calibrate();
        RgbIntrinsicConfig config = load_rgb_intrinsic_config(rgb_intrinsic_config_path_);
        config.width = result.image_width;
        config.height = result.image_height;
        config.fx = static_cast<float>(result.fx);
        config.fy = static_cast<float>(result.fy);
        config.cx = static_cast<float>(result.cx);
        config.cy = static_cast<float>(result.cy);
        config.dist_k1 = static_cast<float>(result.dist_k1);
        config.dist_k2 = static_cast<float>(result.dist_k2);
        config.dist_p1 = static_cast<float>(result.dist_p1);
        config.dist_p2 = static_cast<float>(result.dist_p2);
        config.dist_k3 = static_cast<float>(result.dist_k3);
        if (!is_valid_rgb_intrinsic_config(config)) {
            return {400, "Bad Request", catcheye::http::json_error_body("calibrated RGB intrinsic is out of range")};
        }
        if (!save_rgb_intrinsic_config(config, rgb_intrinsic_config_path_)) {
            return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save RGB intrinsic config file")};
        }
        processor_->update_rgb_intrinsic(config);

        std::ostringstream oss;
        oss << "{"
            << "\"capture_count\":" << rgb_intrinsic_calibrator_->capture_count() << ','
            << "\"rms_error\":" << result.rms_error << ','
            << "\"width\":" << config.width << ','
            << "\"height\":" << config.height << ','
            << "\"fx\":" << config.fx << ','
            << "\"fy\":" << config.fy << ','
            << "\"cx\":" << config.cx << ','
            << "\"cy\":" << config.cy << ','
            << "\"undistort_enabled\":" << (config.undistort_enabled ? "true" : "false") << ','
            << "\"dist_k1\":" << config.dist_k1 << ','
            << "\"dist_k2\":" << config.dist_k2 << ','
            << "\"dist_p1\":" << config.dist_p1 << ','
            << "\"dist_p2\":" << config.dist_p2 << ','
            << "\"dist_k3\":" << config.dist_k3
            << "}";
        return {200, "OK", oss.str()};
    } catch (const std::exception& e) {
        return {400, "Bad Request", catcheye::http::json_error_body(e.what())};
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

catcheye::http::HttpResponse HttpApiServer::handle_get_rgb_cubeeye_extrinsic() const
{
    try {
        return {200, "OK", rgb_cubeeye_extrinsic_config_to_json(load_rgb_cubeeye_extrinsic_config(rgb_cubeeye_extrinsic_config_path_))};
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
}

catcheye::http::HttpResponse HttpApiServer::handle_put_rgb_cubeeye_extrinsic(const std::string& body) const
{
    RgbCubeEyeExtrinsicConfig config;
    try {
        config = load_rgb_cubeeye_extrinsic_config(rgb_cubeeye_extrinsic_config_path_);
    } catch (const std::exception& e) {
        return {500, "Internal Server Error", catcheye::http::json_error_body(e.what())};
    }
    bool has_field = false;
    has_field = parse_float_field(body, "tx_m", config.tx_m) || has_field;
    has_field = parse_float_field(body, "ty_m", config.ty_m) || has_field;
    has_field = parse_float_field(body, "tz_m", config.tz_m) || has_field;
    has_field = parse_float_field(body, "roll_deg", config.roll_deg) || has_field;
    has_field = parse_float_field(body, "pitch_deg", config.pitch_deg) || has_field;
    has_field = parse_float_field(body, "yaw_deg", config.yaw_deg) || has_field;
    has_field = parse_bool_field(body, "cubeeye_distortion_correction_enabled",
                                 config.cubeeye_distortion_correction_enabled) ||
                has_field;
    if (!has_field) {
        return {400, "Bad Request", catcheye::http::json_error_body("invalid RGB CubeEye extrinsic JSON body")};
    }
    if (!is_valid_rgb_cubeeye_extrinsic_config(config)) {
        return {400, "Bad Request", catcheye::http::json_error_body("RGB CubeEye extrinsic out of range")};
    }
    if (!save_rgb_cubeeye_extrinsic_config(config, rgb_cubeeye_extrinsic_config_path_)) {
        return {500, "Internal Server Error", catcheye::http::json_error_body("failed to save RGB CubeEye extrinsic config file")};
    }
    processor_->update_rgb_cubeeye_extrinsic(config);
    return handle_get_rgb_cubeeye_extrinsic();
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
