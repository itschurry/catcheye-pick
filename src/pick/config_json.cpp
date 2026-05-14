#include "pick/config_json.hpp"

#include <cctype>
#include <string>

namespace catcheye::pick {
namespace {

bool field_value(std::string_view body, std::string_view key, std::string& output)
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
    const std::size_t end_pos = body.find_first_of(",}\n", colon_pos + 1U);
    output = trim_json_value(std::string(body.substr(
        colon_pos + 1U,
        end_pos == std::string_view::npos ? std::string_view::npos : end_pos - colon_pos - 1U)));
    return !output.empty();
}

} // namespace

std::string trim_json_value(std::string value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
        value.erase(value.begin());
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
        value.pop_back();
    }
    return value;
}

bool parse_json_bool_field(std::string_view body, std::string_view key, bool& output)
{
    std::string value;
    if (!field_value(body, key, value)) {
        return false;
    }
    if (value == "true") {
        output = true;
        return true;
    }
    if (value == "false") {
        output = false;
        return true;
    }
    return false;
}

bool parse_json_int_field(std::string_view body, std::string_view key, int& output)
{
    std::string value;
    if (!field_value(body, key, value)) {
        return false;
    }
    try {
        std::size_t consumed = 0;
        output = std::stoi(value, &consumed);
        return consumed == value.size();
    } catch (...) {
        return false;
    }
}

bool parse_json_float_field(std::string_view body, std::string_view key, float& output)
{
    std::string value;
    if (!field_value(body, key, value)) {
        return false;
    }
    try {
        std::size_t consumed = 0;
        output = std::stof(value, &consumed);
        return consumed == value.size();
    } catch (...) {
        return false;
    }
}

} // namespace catcheye::pick
