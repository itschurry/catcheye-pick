#include "pick/app.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "CubeEyeCamera.h"
#include "CubeEyeSource.h"
#include "catcheye/input/frame_source.hpp"
#include "catcheye/transport/websocket_publisher.hpp"
#include "pick/cubeye_camera.hpp"
#include "pick/processor.hpp"

namespace catcheye::pick {
namespace {

constexpr std::string_view DEFAULT_CAMERA_PIPELINE =
    "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=10/1,format=NV12 ! videoflip method=rotate-180";
constexpr int CAMERA_READ_SLEEP_MS = 1;

void print_usage() {
    std::cout << "Usage: catcheye-pick [options]\n"
              << "\n"
              << "Options:\n"
              << "  --help                    Show this help\n"
              << "  --version                 Show CubeEye SDK version\n"
              << "  --list-cubeye             List connected CubeEye camera sources\n"
              << "  --detector <name>         Detector backend: ncnn | hailo\n"
              << "  --viewer-only             Start Camera Module 3 and CubeEye without detection\n"
              << "  --ws [port]               Publish viewer-only frames over WebSocket\n"
              << "  --camera-pipeline <pipe>  GStreamer pipeline for Camera Module 3\n"
              << "  --cubeye-frames <list>    CubeEye frames: depth, amplitude, rgb\n";
}

DetectorBackend parse_detector_backend(std::string_view value) {
    if (value == "ncnn") {
        return DetectorBackend::Ncnn;
    }
    if (value == "hailo") {
        return DetectorBackend::Hailo;
    }

    throw std::invalid_argument("unknown detector backend: " + std::string(value));
}

const char* publisher_name(PublisherType type) {
    switch (type) {
    case PublisherType::WebSocket:
        return "websocket";
    case PublisherType::None:
        return "local";
    }
    return "unknown";
}

std::string describe_runtime_mode(const AppOptions& options) {
    const char* processing_name = options.viewer_only ? "viewer only" : "pick detection";
    const char* output_name = options.publisher_type == PublisherType::WebSocket ? "websocket output" : "local output";
    return std::string("camera module 3 + gstreamer source + CubeEye source + ") + processing_name + " + " + output_name;
}

int run_viewer_only(AppBootstrap bootstrap) {
    if (!bootstrap.camera_source || !bootstrap.camera_source->open()) {
        throw std::runtime_error("failed to open Camera Module 3 gstreamer pipeline");
    }

    CubeEyeCameraSession cubeye(bootstrap.processor_config.cubeye_frames);
    cubeye.open();

    PickProcessor processor(std::move(bootstrap.processor_config));
    if (!processor.initialize()) {
        throw std::runtime_error("failed to initialize pick processor");
    }

    catcheye::transport::WebSocketPublisher websocket(bootstrap.websocket_publisher_config);
    if (!websocket.start()) {
        throw std::runtime_error("failed to start WebSocket publisher");
    }

    catcheye::input::Frame camera_frame;
    std::uint64_t frame_index = 0;
    while (true) {
        const auto read_status = bootstrap.camera_source->read(camera_frame);
        if (read_status == catcheye::input::FrameReadStatus::Error) {
            throw std::runtime_error("Camera Module 3 frame read failed");
        }
        if (read_status == catcheye::input::FrameReadStatus::EndOfStream) {
            throw std::runtime_error("Camera Module 3 stream ended");
        }

        ++frame_index;
        const CubeEyeFrameSet cubeye_frames = cubeye.read();
        const PickViewerFrame viewer_frame = processor.process_viewer_frame(camera_frame, cubeye_frames, frame_index);
        const std::string metadata = build_viewer_metadata(viewer_frame);
        const auto payloads = viewer_payload_spans(viewer_frame);
        websocket.publish_payloads(metadata, payloads);
        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_READ_SLEEP_MS));
    }

    return 0;
}

} // namespace

AppOptions parse_app_options(int argc, char** argv) {
    AppOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);

        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--version") {
            options.show_version = true;
        } else if (arg == "--list-cubeye") {
            options.list_cubeye_sources = true;
        } else if (arg == "--viewer-only") {
            options.viewer_only = true;
        } else if (arg == "--ws") {
            if (options.publisher_type != PublisherType::None) {
                throw std::invalid_argument("only one publisher can be selected at a time");
            }
            options.publisher_type = PublisherType::WebSocket;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                options.websocket_port = std::stoi(argv[++i]);
            }
        } else if (arg == "--camera-pipeline") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--camera-pipeline requires a value");
            }
            options.camera_pipeline = argv[++i];
        } else if (arg == "--cubeye-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--cubeye-frames requires a value");
            }
            options.cubeye_frames = argv[++i];
            parse_cubeye_frames(options.cubeye_frames);
        } else if (arg == "--detector") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--detector requires a value");
            }
            options.detector_backend = parse_detector_backend(argv[++i]);
        } else if (arg == "--rtsp") {
            throw std::invalid_argument("--rtsp is not supported by catcheye-pick");
        } else {
            options.positional_args.emplace_back(arg);
        }
    }

    if (options.websocket_port <= 0) {
        throw std::invalid_argument("WebSocket port must be a positive integer");
    }
    if (options.viewer_only && options.publisher_type != PublisherType::WebSocket) {
        throw std::invalid_argument("--viewer-only requires --ws");
    }
    if (!options.viewer_only && options.publisher_type == PublisherType::WebSocket) {
        throw std::invalid_argument("--ws is only supported with --viewer-only");
    }
    if (!options.viewer_only && !options.camera_pipeline.empty()) {
        throw std::invalid_argument("--camera-pipeline is only supported with --viewer-only");
    }
    if (options.viewer_only && !options.positional_args.empty()) {
        throw std::invalid_argument("positional arguments are not used with --viewer-only");
    }

    return options;
}

AppBootstrap build_app_bootstrap(const AppOptions& options) {
    AppBootstrap bootstrap;
    bootstrap.processor_config.detection_enabled = !options.viewer_only;
    bootstrap.processor_config.cubeye_frames = parse_cubeye_frames(options.cubeye_frames);
    bootstrap.publisher_type = options.publisher_type;
    bootstrap.websocket_publisher_config.port = options.websocket_port;

    const std::string camera_pipeline = options.camera_pipeline.empty() ? std::string(DEFAULT_CAMERA_PIPELINE) : options.camera_pipeline;
    bootstrap.camera_source = catcheye::input::create_frame_source(catcheye::input::InputSourceConfig{
        .type = catcheye::input::InputSourceType::Camera,
        .uri = {},
        .camera_pipeline = camera_pipeline,
        .camera_device = {},
        .camera_width = 1280,
        .camera_height = 720,
    });

    return bootstrap;
}

int run_app(int argc, char** argv) {
    const AppOptions options = parse_app_options(argc, argv);

    if (options.show_help) {
        print_usage();
        return 0;
    }

    if (options.show_version) {
        std::cout << "CubeEye SDK " << meere::sensor::last_released_version() << " (" << meere::sensor::last_released_date() << ")\n";
        return 0;
    }

    if (options.list_cubeye_sources) {
        return list_cubeye_sources();
    }

    AppBootstrap bootstrap = build_app_bootstrap(options);
    std::cerr << "catcheye-pick starting (mode='" << describe_runtime_mode(options) << "', publisher='"
              << publisher_name(bootstrap.publisher_type) << "', cubeye_frames='" << options.cubeye_frames << "')\n";

    if (options.viewer_only) {
        return run_viewer_only(std::move(bootstrap));
    }

    print_usage();
    return 0;
}

} // namespace catcheye::pick
