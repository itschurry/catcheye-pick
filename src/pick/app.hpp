#pragma once

#include <memory>
#include <string>
#include <vector>

#include "catcheye/detection/detector_factory.hpp"
#include "catcheye/input/frame_source.hpp"
#include "catcheye/transport/websocket_publisher.hpp"
#include "pick/http_api_server.hpp"
#include "pick/processor_config.hpp"

namespace catcheye::pick {

enum class PublisherType {
    None,
    WebSocket,
};

enum class InputSourceKind {
    Camera,
    Image,
    Video,
};

enum class CameraBackend {
    Realsense,
    IsaacSim,
};

struct AppOptions {
    bool show_help = false;
    bool viewer_only = false;
    PublisherType publisher_type = PublisherType::None;
    int websocket_port = 8080;
    int http_port = 8090;
    InputSourceKind input_source = InputSourceKind::Camera;
    CameraBackend camera_backend = CameraBackend::IsaacSim;
    std::string camera_pipeline;
    std::string depth_pipeline;
    std::string roi_config_path;
    std::string pallet_roi_config_path;
    std::string intrinsics_config_path;
    std::string extrinsics_config_path;
    std::string robot_calibration_config_path;
    int num_threads = 2;
    catcheye::DetectorBackend detector_backend = catcheye::DetectorBackend::Ncnn;
    std::string hef_path;
    std::string metadata_path;
    std::vector<std::string> positional_args;
};

struct AppBootstrap {
    PickProcessorConfig processor_config;
    PublisherType publisher_type = PublisherType::None;
    catcheye::transport::WebSocketPublisherConfig websocket_publisher_config;
    HttpApiServerConfig http_api_server_config;
    std::unique_ptr<catcheye::input::FrameSource> camera_source;
    std::unique_ptr<catcheye::input::FrameSource> depth_source;
    std::string intrinsics_config_path;
    std::string extrinsics_config_path;
    std::string robot_calibration_config_path;
};

AppOptions parse_app_options(int argc, char** argv);
AppBootstrap build_app_bootstrap(const AppOptions& options, const char* executable_path);
int run_app(int argc, char** argv);

} // namespace catcheye::pick
