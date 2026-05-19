#include "pick/calibration_config_repository.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

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

std::string read_file(const std::string& path, std::string_view label)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to load " + std::string(label) + " config: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

} // namespace

bool is_valid_rgb_intrinsic_config(RgbIntrinsicConfig config)
{
    return config.width > 0 && config.height > 0 &&
           std::isfinite(config.fx) && std::isfinite(config.fy) && config.fx > 0.0F && config.fy > 0.0F &&
           std::isfinite(config.cx) && std::isfinite(config.cy) &&
           std::isfinite(config.dist_k1) && std::isfinite(config.dist_k2) && std::isfinite(config.dist_p1) &&
           std::isfinite(config.dist_p2) && std::isfinite(config.dist_k3);
}

RgbIntrinsicConfig load_rgb_intrinsic_config(const std::string& path)
{
    RgbIntrinsicConfig config;
    const std::string json = read_file(path, "RGB intrinsic");
    float width = static_cast<float>(config.width);
    float height = static_cast<float>(config.height);
    if (parse_float_field(json, "width", width)) {
        config.width = static_cast<int>(width);
    }
    if (parse_float_field(json, "height", height)) {
        config.height = static_cast<int>(height);
    }
    parse_float_field(json, "fx", config.fx);
    parse_float_field(json, "fy", config.fy);
    parse_float_field(json, "cx", config.cx);
    parse_float_field(json, "cy", config.cy);
    parse_bool_field(json, "undistort_enabled", config.undistort_enabled);
    parse_float_field(json, "dist_k1", config.dist_k1);
    parse_float_field(json, "dist_k2", config.dist_k2);
    parse_float_field(json, "dist_p1", config.dist_p1);
    parse_float_field(json, "dist_p2", config.dist_p2);
    parse_float_field(json, "dist_k3", config.dist_k3);
    if (!is_valid_rgb_intrinsic_config(config)) {
        throw std::runtime_error("invalid RGB intrinsic config: " + path);
    }
    return config;
}

std::string rgb_intrinsic_config_to_json(RgbIntrinsicConfig config)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"width\": " << config.width << ",\n"
        << "  \"height\": " << config.height << ",\n"
        << "  \"fx\": " << config.fx << ",\n"
        << "  \"fy\": " << config.fy << ",\n"
        << "  \"cx\": " << config.cx << ",\n"
        << "  \"cy\": " << config.cy << ",\n"
        << "  \"undistort_enabled\": " << (config.undistort_enabled ? "true" : "false") << ",\n"
        << "  \"dist_k1\": " << config.dist_k1 << ",\n"
        << "  \"dist_k2\": " << config.dist_k2 << ",\n"
        << "  \"dist_p1\": " << config.dist_p1 << ",\n"
        << "  \"dist_p2\": " << config.dist_p2 << ",\n"
        << "  \"dist_k3\": " << config.dist_k3 << "\n"
        << "}\n";
    return oss.str();
}

bool save_rgb_intrinsic_config(RgbIntrinsicConfig config, const std::string& path)
{
    if (!is_valid_rgb_intrinsic_config(config)) {
        return false;
    }

    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        return false;
    }
    output << rgb_intrinsic_config_to_json(config);
    return output.good();
}

bool is_valid_rgb_cubeeye_extrinsic_config(RgbCubeEyeExtrinsicConfig config)
{
    return std::isfinite(config.tx_m) && std::isfinite(config.ty_m) && std::isfinite(config.tz_m) &&
           std::isfinite(config.roll_deg) && std::isfinite(config.pitch_deg) && std::isfinite(config.yaw_deg);
}

RgbCubeEyeExtrinsicConfig load_rgb_cubeeye_extrinsic_config(const std::string& path)
{
    RgbCubeEyeExtrinsicConfig config;
    const std::string json = read_file(path, "RGB CubeEye extrinsic");
    parse_float_field(json, "tx_m", config.tx_m);
    parse_float_field(json, "ty_m", config.ty_m);
    parse_float_field(json, "tz_m", config.tz_m);
    parse_float_field(json, "roll_deg", config.roll_deg);
    parse_float_field(json, "pitch_deg", config.pitch_deg);
    parse_float_field(json, "yaw_deg", config.yaw_deg);
    if (!is_valid_rgb_cubeeye_extrinsic_config(config)) {
        throw std::runtime_error("invalid RGB CubeEye extrinsic config: " + path);
    }
    return config;
}

std::string rgb_cubeeye_extrinsic_config_to_json(RgbCubeEyeExtrinsicConfig config)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"tx_m\": " << config.tx_m << ",\n"
        << "  \"ty_m\": " << config.ty_m << ",\n"
        << "  \"tz_m\": " << config.tz_m << ",\n"
        << "  \"roll_deg\": " << config.roll_deg << ",\n"
        << "  \"pitch_deg\": " << config.pitch_deg << ",\n"
        << "  \"yaw_deg\": " << config.yaw_deg << "\n"
        << "}\n";
    return oss.str();
}

bool save_rgb_cubeeye_extrinsic_config(RgbCubeEyeExtrinsicConfig config, const std::string& path)
{
    if (!is_valid_rgb_cubeeye_extrinsic_config(config)) {
        return false;
    }

    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        return false;
    }
    output << rgb_cubeeye_extrinsic_config_to_json(config);
    return output.good();
}

} // namespace catcheye::pick
