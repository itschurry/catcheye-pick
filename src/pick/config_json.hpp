#pragma once

#include <string>
#include <string_view>

namespace catcheye::pick {

std::string trim_json_value(std::string value);
bool parse_json_bool_field(std::string_view body, std::string_view key, bool& output);
bool parse_json_int_field(std::string_view body, std::string_view key, int& output);
bool parse_json_float_field(std::string_view body, std::string_view key, float& output);

} // namespace catcheye::pick
