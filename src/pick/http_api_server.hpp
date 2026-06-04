#pragma once

#include <memory>
#include <string>

#include "catcheye/http/http_server.hpp"

namespace catcheye::pick {

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
        std::string intrinsics_config_path,
        std::string extrinsics_config_path,
        std::string robot_calibration_config_path,
        std::string object_catalog_config_path,
        PickProcessor* processor);
    ~HttpApiServer();

    bool start();
    void stop();

  private:
    catcheye::http::HttpResponse handle_get_intrinsics() const;
    catcheye::http::HttpResponse handle_get_extrinsics() const;
    catcheye::http::HttpResponse handle_get_objects() const;
    catcheye::http::HttpResponse handle_get_pose_estimates() const;
    catcheye::http::HttpResponse handle_put_pose_estimates(const std::string& body) const;
    catcheye::http::HttpResponse handle_get_robot_calibration() const;
    catcheye::http::HttpResponse handle_put_robot_calibration(const std::string& body) const;

    HttpApiServerConfig config_;
    std::string roi_config_path_;
    std::string pallet_roi_config_path_;
    std::string intrinsics_config_path_;
    std::string extrinsics_config_path_;
    std::string robot_calibration_config_path_;
    std::string object_catalog_config_path_;
    PickProcessor* processor_ = nullptr;
    std::unique_ptr<catcheye::http::HttpServer> server_;
};

} // namespace catcheye::pick
