#pragma once

#include <memory>
#include <string>

#include "catcheye/http/http_server.hpp"

namespace catcheye::pick {

class CubeEyeCameraSession;
class PickProcessor;

struct HttpApiServerConfig {
    std::string bind_address = "0.0.0.0";
    int port = 8090;
};

class HttpApiServer final {
  public:
    HttpApiServer(
        HttpApiServerConfig config,
        std::string roi_config_path,
        std::string pallet_roi_config_path,
        std::string rgb_cubeeye_offset_config_path,
        std::string pallet_candidate_config_path,
        std::string pointcloud_roi_config_path,
        std::string robot_calibration_config_path,
        PickProcessor* processor,
        CubeEyeCameraSession* cubeeye);
    ~HttpApiServer();

    bool start();
    void stop();

  private:
    catcheye::http::HttpResponse handle_get_cubeeye_properties() const;
    catcheye::http::HttpResponse handle_put_cubeeye_property(const std::string& key, const std::string& body) const;
    catcheye::http::HttpResponse handle_get_rgb_cubeeye_offset() const;
    catcheye::http::HttpResponse handle_put_rgb_cubeeye_offset(const std::string& body) const;
    catcheye::http::HttpResponse handle_get_pallet_candidate_config() const;
    catcheye::http::HttpResponse handle_put_pallet_candidate_config(const std::string& body) const;
    catcheye::http::HttpResponse handle_get_pointcloud_roi_config() const;
    catcheye::http::HttpResponse handle_put_pointcloud_roi_config(const std::string& body) const;
    catcheye::http::HttpResponse handle_get_robot_calibration() const;
    catcheye::http::HttpResponse handle_put_robot_calibration(const std::string& body) const;

    HttpApiServerConfig config_;
    std::string roi_config_path_;
    std::string pallet_roi_config_path_;
    std::string rgb_cubeeye_offset_config_path_;
    std::string pallet_candidate_config_path_;
    std::string pointcloud_roi_config_path_;
    std::string robot_calibration_config_path_;
    PickProcessor* processor_ = nullptr;
    CubeEyeCameraSession* cubeeye_ = nullptr;
    std::unique_ptr<catcheye::http::HttpServer> server_;
};

} // namespace catcheye::pick
