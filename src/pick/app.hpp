#pragma once

#include <memory>
#include <string>
#include <vector>

#include "catcheye/input/frame_source.hpp"
#include "catcheye/transport/websocket_publisher.hpp"
#include "pick/processor_config.hpp"

namespace catcheye::pick {

enum class PublisherType {
    None,
    WebSocket,
};

enum class DetectorBackend {
    Ncnn,
    Hailo,
};

struct AppOptions {
    bool show_help = false;
    bool show_version = false;
    bool list_cubeeye_sources = false;
    bool viewer_only = false;
    PublisherType publisher_type = PublisherType::None;
    int websocket_port = 8080;
    std::string camera_pipeline;
    std::string cubeeye_frames = "depth,amplitude";
    DetectorBackend detector_backend = DetectorBackend::Ncnn;
    std::vector<std::string> positional_args;
};

struct AppBootstrap {
    PickProcessorConfig processor_config;
    PublisherType publisher_type = PublisherType::None;
    catcheye::transport::WebSocketPublisherConfig websocket_publisher_config;
    std::unique_ptr<catcheye::input::FrameSource> camera_source;
};

AppOptions parse_app_options(int argc, char** argv);
AppBootstrap build_app_bootstrap(const AppOptions& options);
int run_app(int argc, char** argv);

} // namespace catcheye::pick
