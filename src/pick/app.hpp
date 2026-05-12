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

enum class CameraInputMode {
    RgbCubeEye,
    RgbOnly,
    CubeEyeOnly,
};

struct AppOptions {
    bool show_help = false;
    bool show_version = false;
    bool list_cubeeye_sources = false;
    bool viewer_only = false;
    PublisherType publisher_type = PublisherType::None;
    int websocket_port = 8080;
    int http_port = 8090;
    CameraInputMode camera_input_mode = CameraInputMode::RgbCubeEye;
    std::string camera_pipeline;
    std::string cubeeye_frames = "depth,amplitude";
    std::string roi_config_path;
    std::string pallet_roi_config_path;
    int cubeeye_camera_fps = 0;
    int pointcloud_downsample = 4;
    int num_threads = 2;
    bool camera_pipeline_set = false;
    bool cubeeye_frames_set = false;
    bool cubeeye_camera_fps_set = false;
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
    int cubeeye_camera_fps = 0;
};

AppOptions parse_app_options(int argc, char** argv);
AppBootstrap build_app_bootstrap(const AppOptions& options, const char* executable_path);
int run_app(int argc, char** argv);

} // namespace catcheye::pick
