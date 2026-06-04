#include "pick/config_json.hpp"

#include <cctype>
#include <sstream>
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

bool parse_json_string_field(std::string_view body, std::string_view key, std::string& output)
{
    std::string value;
    if (!field_value(body, key, value) || value.size() < 2U || value.front() != '"' || value.back() != '"') {
        return false;
    }
    output = value.substr(1U, value.size() - 2U);
    return true;
}

bool parse_json_float_array_field(std::string_view body, std::string_view key, std::vector<float>& output)
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
    const std::size_t open_pos = body.find('[', colon_pos + 1U);
    const std::size_t close_pos = body.find(']', open_pos + 1U);
    if (open_pos == std::string_view::npos || close_pos == std::string_view::npos) {
        return false;
    }

    std::stringstream stream(std::string(body.substr(open_pos + 1U, close_pos - open_pos - 1U)));
    std::string token;
    std::vector<float> values;
    while (std::getline(stream, token, ',')) {
        token = trim_json_value(token);
        if (token.empty()) {
            return false;
        }
        try {
            std::size_t consumed = 0;
            values.push_back(std::stof(token, &consumed));
            if (consumed != token.size()) {
                return false;
            }
        } catch (...) {
            return false;
        }
    }
    output = std::move(values);
    return true;
}

} // namespace catcheye::pick
