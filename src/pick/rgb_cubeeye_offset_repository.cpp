#include "pick/rgb_cubeeye_offset_repository.hpp"

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

} // namespace

bool is_valid_rgb_cubeeye_offset(RgbCubeEyeOffset offset)
{
    return std::isfinite(offset.u) && std::isfinite(offset.v) && offset.u >= -1.0F && offset.u <= 1.0F &&
           offset.v >= -1.0F && offset.v <= 1.0F &&
           std::isfinite(offset.tx_m) && std::isfinite(offset.ty_m) && std::isfinite(offset.tz_m) &&
           std::isfinite(offset.roll_deg) && std::isfinite(offset.pitch_deg) && std::isfinite(offset.yaw_deg) &&
           offset.rgb_width > 0 && offset.rgb_height > 0 &&
           std::isfinite(offset.rgb_fx) && std::isfinite(offset.rgb_fy) && offset.rgb_fx > 0.0F && offset.rgb_fy > 0.0F &&
           std::isfinite(offset.rgb_cx) && std::isfinite(offset.rgb_cy) &&
           std::isfinite(offset.rgb_dist_k1) && std::isfinite(offset.rgb_dist_k2) && std::isfinite(offset.rgb_dist_p1) &&
           std::isfinite(offset.rgb_dist_p2) && std::isfinite(offset.rgb_dist_k3);
}

RgbCubeEyeOffset load_rgb_cubeeye_offset_config(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to load RGB CubeEye offset config: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    RgbCubeEyeOffset offset;
    const std::string json = buffer.str();
    parse_float_field(json, "u", offset.u);
    parse_float_field(json, "v", offset.v);
    parse_float_field(json, "tx_m", offset.tx_m);
    parse_float_field(json, "ty_m", offset.ty_m);
    parse_float_field(json, "tz_m", offset.tz_m);
    parse_float_field(json, "roll_deg", offset.roll_deg);
    parse_float_field(json, "pitch_deg", offset.pitch_deg);
    parse_float_field(json, "yaw_deg", offset.yaw_deg);
    float rgb_width = static_cast<float>(offset.rgb_width);
    float rgb_height = static_cast<float>(offset.rgb_height);
    if (parse_float_field(json, "rgb_width", rgb_width)) {
        offset.rgb_width = static_cast<int>(rgb_width);
    }
    if (parse_float_field(json, "rgb_height", rgb_height)) {
        offset.rgb_height = static_cast<int>(rgb_height);
    }
    parse_float_field(json, "rgb_fx", offset.rgb_fx);
    parse_float_field(json, "rgb_fy", offset.rgb_fy);
    parse_float_field(json, "rgb_cx", offset.rgb_cx);
    parse_float_field(json, "rgb_cy", offset.rgb_cy);
    parse_bool_field(json, "rgb_undistort_enabled", offset.rgb_undistort_enabled);
    parse_float_field(json, "rgb_dist_k1", offset.rgb_dist_k1);
    parse_float_field(json, "rgb_dist_k2", offset.rgb_dist_k2);
    parse_float_field(json, "rgb_dist_p1", offset.rgb_dist_p1);
    parse_float_field(json, "rgb_dist_p2", offset.rgb_dist_p2);
    parse_float_field(json, "rgb_dist_k3", offset.rgb_dist_k3);
    if (!is_valid_rgb_cubeeye_offset(offset)) {
        throw std::runtime_error("invalid RGB CubeEye offset config: " + path);
    }
    return offset;
}

std::string rgb_cubeeye_offset_to_json(RgbCubeEyeOffset offset)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"tx_m\": " << offset.tx_m << ",\n"
        << "  \"ty_m\": " << offset.ty_m << ",\n"
        << "  \"tz_m\": " << offset.tz_m << ",\n"
        << "  \"roll_deg\": " << offset.roll_deg << ",\n"
        << "  \"pitch_deg\": " << offset.pitch_deg << ",\n"
        << "  \"yaw_deg\": " << offset.yaw_deg << ",\n"
        << "  \"rgb_width\": " << offset.rgb_width << ",\n"
        << "  \"rgb_height\": " << offset.rgb_height << ",\n"
        << "  \"rgb_fx\": " << offset.rgb_fx << ",\n"
        << "  \"rgb_fy\": " << offset.rgb_fy << ",\n"
        << "  \"rgb_cx\": " << offset.rgb_cx << ",\n"
        << "  \"rgb_cy\": " << offset.rgb_cy << ",\n"
        << "  \"rgb_undistort_enabled\": " << (offset.rgb_undistort_enabled ? "true" : "false") << ",\n"
        << "  \"rgb_dist_k1\": " << offset.rgb_dist_k1 << ",\n"
        << "  \"rgb_dist_k2\": " << offset.rgb_dist_k2 << ",\n"
        << "  \"rgb_dist_p1\": " << offset.rgb_dist_p1 << ",\n"
        << "  \"rgb_dist_p2\": " << offset.rgb_dist_p2 << ",\n"
        << "  \"rgb_dist_k3\": " << offset.rgb_dist_k3 << "\n"
        << "}\n";
    return oss.str();
}

bool save_rgb_cubeeye_offset_config(RgbCubeEyeOffset offset, const std::string& path)
{
    if (!is_valid_rgb_cubeeye_offset(offset)) {
        return false;
    }

    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        return false;
    }
    output << rgb_cubeeye_offset_to_json(offset);
    return output.good();
}

} // namespace catcheye::pick
