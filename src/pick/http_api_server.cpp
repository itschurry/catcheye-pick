#include "pick/http_api_server.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "catcheye/roi/roi_repository.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {
namespace {

constexpr int SOCKET_ENABLE = 1;
constexpr int POLL_TIMEOUT_MS = 200;
constexpr std::size_t MAX_REQUEST_BYTES = 1024 * 1024;

struct JsonValue {
    enum class Type {
        Boolean,
        Integer,
    };

    Type type = Type::Integer;
    bool bool_value = false;
    int int_value = 0;
};

std::string escape_json_string(std::string_view value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
        case '"':
            escaped += "\\\"";
            break;
        case '\\':
            escaped += "\\\\";
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            escaped.push_back(ch);
            break;
        }
    }
    return escaped;
}

bool send_all(int sock_fd, const void* data, std::size_t size)
{
    const auto* bytes = static_cast<const std::byte*>(data);
    std::span<const std::byte> remaining{bytes, size};

    while (!remaining.empty()) {
        const ssize_t written = ::send(sock_fd, remaining.data(), remaining.size(), MSG_NOSIGNAL);
        if (written <= 0) {
            return false;
        }
        remaining = remaining.subspan(static_cast<std::size_t>(written));
    }

    return true;
}

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

std::string header_value(std::string_view request, std::string_view header_name)
{
    const std::string needle = std::string(header_name) + ":";
    std::istringstream iss{std::string(request)};
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.size() <= needle.size()) {
            continue;
        }

        std::string prefix = line.substr(0, needle.size());
        std::string needle_lower = needle;
        std::transform(prefix.begin(), prefix.end(), prefix.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        std::transform(needle_lower.begin(), needle_lower.end(), needle_lower.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (prefix == needle_lower) {
            return trim(line.substr(needle.size()));
        }
    }

    return {};
}

std::string json_error_body(std::string_view message)
{
    return "{\"error\":\"" + escape_json_string(message) + "\"}";
}

std::string json_error_body(std::string_view message, const std::vector<std::string>& details)
{
    std::ostringstream oss;
    oss << "{\"error\":\"" << escape_json_string(message) << "\"";
    if (!details.empty()) {
        oss << ",\"details\":[";
        for (std::size_t i = 0; i < details.size(); ++i) {
            if (i > 0) {
                oss << ',';
            }
            oss << "\"" << escape_json_string(details[i]) << "\"";
        }
        oss << "]";
    }
    oss << "}";
    return oss.str();
}

std::vector<std::string> validation_issue_messages(const catcheye::roi::ValidationResult& validation)
{
    std::vector<std::string> details;
    details.reserve(validation.issues.size());
    for (const auto& issue : validation.issues) {
        std::ostringstream oss;
        oss << "zone_index=" << issue.zone_index
            << ", point_index=" << issue.point_index
            << ", message=" << issue.message;
        details.push_back(oss.str());
    }
    return details;
}

bool parse_request_line(std::string_view request, std::string& method, std::string& path, std::string& version)
{
    const std::size_t line_end = request.find("\r\n");
    if (line_end == std::string_view::npos) {
        return false;
    }

    std::istringstream iss{std::string(request.substr(0, line_end))};
    return static_cast<bool>(iss >> method >> path >> version);
}

bool read_http_request(int client_fd, std::string& request, std::string& body)
{
    request.clear();
    body.clear();

    std::array<char, 4096> buffer{};
    std::size_t header_end = std::string::npos;
    while (request.size() < MAX_REQUEST_BYTES) {
        const ssize_t received = ::recv(client_fd, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            return false;
        }

        request.append(buffer.data(), static_cast<std::size_t>(received));
        header_end = request.find("\r\n\r\n");
        if (header_end != std::string::npos) {
            break;
        }
    }

    if (header_end == std::string::npos) {
        return false;
    }

    const std::string headers = request.substr(0, header_end + 4U);
    std::size_t content_length = 0;
    const std::string content_length_text = header_value(headers, "Content-Length");
    if (!content_length_text.empty()) {
        try {
            content_length = static_cast<std::size_t>(std::stoul(content_length_text));
        } catch (...) {
            return false;
        }
    }

    body = request.substr(header_end + 4U);
    while (body.size() < content_length && request.size() < MAX_REQUEST_BYTES) {
        const ssize_t received = ::recv(client_fd, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            return false;
        }
        request.append(buffer.data(), static_cast<std::size_t>(received));
        body.append(buffer.data(), static_cast<std::size_t>(received));
    }

    if (body.size() > content_length) {
        body.resize(content_length);
    }
    return body.size() == content_length;
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
    if (running_) {
        return true;
    }
    if (processor_ == nullptr || config_.port <= 0) {
        return false;
    }

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        return false;
    }

    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &SOCKET_ENABLE, sizeof(SOCKET_ENABLE));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(config_.port));
    addr.sin_addr.s_addr = config_.bind_address == "0.0.0.0" ? INADDR_ANY : inet_addr(config_.bind_address.c_str());

    if (::bind(server_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (::listen(server_fd_, 8) != 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&HttpApiServer::accept_loop, this);
    std::cerr << "HTTP API listening on " << config_.bind_address << ':' << config_.port << '\n';
    return true;
}

void HttpApiServer::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;
    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
}

void HttpApiServer::accept_loop()
{
    pollfd pfd{};
    pfd.fd = server_fd_;
    pfd.events = POLLIN;

    while (running_) {
        const int poll_result = ::poll(&pfd, 1, POLL_TIMEOUT_MS);
        if (poll_result <= 0) {
            continue;
        }

        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            continue;
        }

        handle_client(client_fd);
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
    }
}

void HttpApiServer::handle_client(int client_fd)
{
    std::string request;
    std::string body;
    if (!read_http_request(client_fd, request, body)) {
        send_response(client_fd, 400, "Bad Request", json_error_body("failed to read HTTP request"));
        return;
    }

    std::string method;
    std::string path;
    std::string version;
    if (!parse_request_line(request, method, path, version)) {
        send_response(client_fd, 400, "Bad Request", json_error_body("invalid HTTP request line"));
        return;
    }

    if (path == "/api/roi" || path == "/api/pallet-roi") {
        const RoiConfigKind kind = path == "/api/pallet-roi" ? RoiConfigKind::Pallet : RoiConfigKind::Person;
        if (method == "GET") {
            handle_get_roi(client_fd, kind);
            return;
        }
        if (method == "PUT") {
            handle_put_roi(client_fd, body, kind);
            return;
        }
        send_response(client_fd, 405, "Method Not Allowed", json_error_body("method not allowed"));
        return;
    }

    if (path == "/api/cubeeye/properties") {
        if (method == "GET") {
            handle_get_cubeeye_properties(client_fd);
            return;
        }
        send_response(client_fd, 405, "Method Not Allowed", json_error_body("method not allowed"));
        return;
    }

    constexpr std::string_view property_prefix = "/api/cubeeye/properties/";
    if (path.rfind(std::string(property_prefix), 0) == 0) {
        const std::string key = path.substr(property_prefix.size());
        if (method == "PUT") {
            handle_put_cubeeye_property(client_fd, key, body);
            return;
        }
        send_response(client_fd, 405, "Method Not Allowed", json_error_body("method not allowed"));
        return;
    }

    send_response(client_fd, 404, "Not Found", json_error_body("unknown endpoint"));
}

bool HttpApiServer::send_response(int client_fd, int status_code, const std::string& status_text, const std::string& body) const
{
    std::ostringstream oss;
    oss << "HTTP/1.1 " << status_code << ' ' << status_text << "\r\n"
        << "Content-Type: application/json\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Connection: close\r\n\r\n"
        << body;
    const std::string response = oss.str();
    return send_all(client_fd, response.data(), response.size());
}

const std::string& HttpApiServer::roi_config_path(RoiConfigKind kind) const
{
    return kind == RoiConfigKind::Pallet ? pallet_roi_config_path_ : roi_config_path_;
}

bool HttpApiServer::handle_get_roi(int client_fd, RoiConfigKind kind)
{
    const auto parse_result = catcheye::roi::RoiRepository::load_from_file(roi_config_path(kind));
    if (!parse_result.success) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body("failed to load ROI config file", parse_result.errors));
    }

    const auto validation = catcheye::roi::validate_camera_roi_config(parse_result.config);
    if (!validation.valid) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body("ROI config file is invalid", validation_issue_messages(validation)));
    }

    return send_response(client_fd, 200, "OK", catcheye::roi::RoiRepository::to_json_string(parse_result.config, 2));
}

bool HttpApiServer::handle_put_roi(int client_fd, const std::string& body, RoiConfigKind kind)
{
    const auto parse_result = catcheye::roi::RoiRepository::from_json_string(body);
    if (!parse_result.success) {
        return send_response(client_fd, 400, "Bad Request", json_error_body("failed to parse ROI JSON", parse_result.errors));
    }

    const auto validation = catcheye::roi::validate_camera_roi_config(parse_result.config);
    if (!validation.valid) {
        return send_response(client_fd, 400, "Bad Request", json_error_body("ROI config failed validation", validation_issue_messages(validation)));
    }

    if (!catcheye::roi::RoiRepository::save_to_file(parse_result.config, roi_config_path(kind))) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body("failed to save ROI config file"));
    }

    const bool updated = kind == RoiConfigKind::Pallet
        ? processor_->update_pallet_roi_config(parse_result.config)
        : processor_->update_roi_config(parse_result.config);
    if (!updated) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body("failed to apply ROI config in memory"));
    }

    return send_response(client_fd, 200, "OK", catcheye::roi::RoiRepository::to_json_string(parse_result.config, 2));
}

bool HttpApiServer::handle_get_cubeeye_properties(int client_fd)
{
    if (cubeeye_ == nullptr) {
        return send_response(client_fd, 409, "Conflict", json_error_body("CubeEye is not enabled"));
    }
    try {
        const auto properties = cubeeye_->properties_json();
        if (!properties.has_value()) {
            return send_response(client_fd, 409, "Conflict", json_error_body("CubeEye is not running"));
        }
        return send_response(client_fd, 200, "OK", *properties);
    } catch (const std::exception& e) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body(e.what()));
    }
}

bool HttpApiServer::handle_put_cubeeye_property(int client_fd, const std::string& key, const std::string& body)
{
    if (cubeeye_ == nullptr) {
        return send_response(client_fd, 409, "Conflict", json_error_body("CubeEye is not enabled"));
    }
    if (!is_supported_cubeeye_property(key)) {
        return send_response(client_fd, 400, "Bad Request", json_error_body("unsupported CubeEye property"));
    }

    JsonValue value;
    if (!parse_value_body(body, value)) {
        return send_response(client_fd, 400, "Bad Request", json_error_body("invalid property JSON body"));
    }

    bool updated = false;
    if (is_bool_cubeeye_property(key)) {
        if (value.type != JsonValue::Type::Boolean) {
            return send_response(client_fd, 400, "Bad Request", json_error_body("property value must be boolean"));
        }
        updated = cubeeye_->set_bool_property(key, value.bool_value);
    } else {
        if (value.type != JsonValue::Type::Integer) {
            return send_response(client_fd, 400, "Bad Request", json_error_body("property value must be integer"));
        }
        if (!valid_int_value(key, value.int_value)) {
            return send_response(client_fd, 400, "Bad Request", json_error_body("property value out of range"));
        }
        updated = cubeeye_->set_int_property(key, value.int_value);
    }

    if (!updated) {
        return send_response(client_fd, 500, "Internal Server Error", json_error_body("failed to set CubeEye property"));
    }
    return handle_get_cubeeye_properties(client_fd);
}

} // namespace catcheye::pick
