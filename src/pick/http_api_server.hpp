#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace catcheye::pick {

class CubeEyeCameraSession;
class PickProcessor;

struct HttpApiServerConfig {
    std::string bind_address = "0.0.0.0";
    int port = 8090;
};

enum class RoiConfigKind {
    Person,
    Pallet,
};

class HttpApiServer final {
  public:
    HttpApiServer(
        HttpApiServerConfig config,
        std::string roi_config_path,
        std::string pallet_roi_config_path,
        PickProcessor* processor,
        CubeEyeCameraSession* cubeeye);
    ~HttpApiServer();

    bool start();
    void stop();

  private:
    void accept_loop();
    void handle_client(int client_fd);
    bool send_response(int client_fd, int status_code, const std::string& status_text, const std::string& body) const;
    bool handle_get_roi(int client_fd, RoiConfigKind kind);
    bool handle_put_roi(int client_fd, const std::string& body, RoiConfigKind kind);
    bool handle_get_cubeeye_properties(int client_fd);
    bool handle_put_cubeeye_property(int client_fd, const std::string& key, const std::string& body);
    const std::string& roi_config_path(RoiConfigKind kind) const;

    HttpApiServerConfig config_;
    std::string roi_config_path_;
    std::string pallet_roi_config_path_;
    PickProcessor* processor_ = nullptr;
    CubeEyeCameraSession* cubeeye_ = nullptr;
    int server_fd_ = -1;
    std::atomic<bool> running_ = false;
    std::thread accept_thread_;
};

} // namespace catcheye::pick
