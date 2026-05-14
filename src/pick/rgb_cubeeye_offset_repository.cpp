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

} // namespace

bool is_valid_rgb_cubeeye_offset(RgbCubeEyeOffset offset)
{
    return std::isfinite(offset.u) && std::isfinite(offset.v) && offset.u >= -1.0F && offset.u <= 1.0F &&
           offset.v >= -1.0F && offset.v <= 1.0F;
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
    if (!parse_float_field(json, "u", offset.u) || !parse_float_field(json, "v", offset.v) ||
        !is_valid_rgb_cubeeye_offset(offset)) {
        throw std::runtime_error("invalid RGB CubeEye offset config: " + path);
    }
    return offset;
}

std::string rgb_cubeeye_offset_to_json(RgbCubeEyeOffset offset)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"u\": " << offset.u << ",\n"
        << "  \"v\": " << offset.v << "\n"
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
