#include "pick/app.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "catcheye/input/frame_source.hpp"
#include "catcheye/roi/roi_repository.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "catcheye/transport/websocket_publisher.hpp"
#include "catcheye/visualization/annotation_renderer.hpp"
#include "pick/http_api_server.hpp"
#include "pick/processor.hpp"
#include "pick/robot_calibration_repository.hpp"
#include "pick/viewer_metadata.hpp"

namespace catcheye::pick {
namespace {

constexpr std::string_view DEFAULT_CAMERA_PIPELINE =
    "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=10/1,format=NV12 ! videoflip method=rotate-180";
constexpr int CAMERA_READ_SLEEP_MS = 1;

void print_usage()
{
    std::cout << "Usage: catcheye-pick [options]\n"
              << "\n"
              << "Options:\n"
              << "  --help                    Show this help\n"
              << "  --detector <name>         Detector backend: ncnn | hailo\n"
              << "  --hef <path>              Hailo HEF model path\n"
              << "  --metadata <path>         Detector metadata YAML path\n"
              << "  --num-threads <count>     NCNN inference threads (default: 2)\n"
              << "  --camera-input <mode>     Camera input: rgb\n"
              << "  --viewer-only             Start camera input without detection\n"
              << "  --ws [port]               Publish frames over WebSocket\n"
              << "  --http-port <port>        HTTP API port (default: 8090)\n"
              << "  --camera-pipeline <pipe>  GStreamer RGB camera pipeline\n"
              << "  --depth-pipeline <pipe>   GStreamer depth visualization pipeline\n"
              << "  --roi <path>              Person ROI config path\n"
              << "  --pallet-roi <path>       Pallet ROI config path\n"
              << "  --intrinsics <path>       Camera intrinsics JSON path\n"
              << "  --extrinsics <path>       Camera extrinsics JSON path\n"
              << "  --robot-calibration <path>  Robot calibration config path\n";
}

catcheye::DetectorBackend parse_detector_backend(std::string_view value)
{
    if (value == "ncnn") {
        return catcheye::DetectorBackend::Ncnn;
    }
    if (value == "hailo") {
        return catcheye::DetectorBackend::Hailo;
    }
    throw std::invalid_argument("unknown detector backend: " + std::string(value));
}

RgbdSourceProfile parse_source_profile(std::string_view value)
{
    if (value == "rgb") {
        return RgbdSourceProfile::RgbOnly;
    }
    throw std::invalid_argument("unknown camera input mode: " + std::string(value));
}

const char* publisher_name(PublisherType type)
{
    switch (type) {
    case PublisherType::WebSocket:
        return "websocket";
    case PublisherType::None:
        return "local";
    }
    return "unknown";
}

const char* detector_backend_name(catcheye::DetectorBackend backend)
{
    switch (backend) {
    case catcheye::DetectorBackend::Ncnn:
        return "ncnn";
    case catcheye::DetectorBackend::Hailo:
        return "hailo";
    }
    return "unknown";
}

std::string resolve_default_config_path(const char* executable_path, std::string_view filename)
{
    const std::filesystem::path executable = executable_path ? std::filesystem::path(executable_path) : std::filesystem::path{};
    const std::filesystem::path install_root = executable.has_parent_path() ? executable.parent_path().parent_path() : std::filesystem::current_path();
    return (install_root / "config" / filename).string();
}

std::string resolve_default_model_path(const char* executable_path, std::string_view filename)
{
    const std::filesystem::path executable = executable_path ? std::filesystem::path(executable_path) : std::filesystem::path{};
    const std::filesystem::path install_root = executable.has_parent_path() ? executable.parent_path().parent_path() : std::filesystem::current_path();
    return (install_root / "models" / filename).string();
}

catcheye::roi::CameraRoiConfig load_roi_config(const std::string& path)
{
    const auto parse_result = catcheye::roi::RoiRepository::load_from_file(path);
    if (!parse_result.success) {
        throw std::runtime_error("failed to load ROI config: " + path);
    }
    const auto validation = catcheye::roi::validate_camera_roi_config(parse_result.config);
    if (!validation.valid) {
        throw std::runtime_error("ROI config failed validation: " + path);
    }
    return parse_result.config;
}

std::string describe_runtime_mode(const AppOptions& options)
{
    const char* processing_name = options.viewer_only ? "viewer only" : "pick detection";
    const char* output_name = options.publisher_type == PublisherType::WebSocket ? "websocket output" : "local output";
    return std::string("RGB camera + ") + processing_name + " + " + output_name;
}

void start_http_api(AppBootstrap& bootstrap, PickProcessor& processor, std::optional<HttpApiServer>& http_api_server)
{
    http_api_server.emplace(
        bootstrap.http_api_server_config,
        bootstrap.processor_config.roi_config_path,
        bootstrap.processor_config.pallet_roi_config_path,
        bootstrap.intrinsics_config_path,
        bootstrap.extrinsics_config_path,
        bootstrap.robot_calibration_config_path,
        &processor);
    if (!http_api_server->start()) {
        throw std::runtime_error("failed to start HTTP API server");
    }
}

int run_viewer_only(AppBootstrap bootstrap)
{
    if (bootstrap.camera_source == nullptr) {
        throw std::runtime_error("viewer-only requires RGB camera input");
    }
    if (!bootstrap.camera_source->open()) {
        throw std::runtime_error("failed to open RGB camera gstreamer pipeline");
    }
    if (bootstrap.depth_source != nullptr && !bootstrap.depth_source->open()) {
        throw std::runtime_error("failed to open depth gstreamer pipeline");
    }

    PickProcessor processor(std::move(bootstrap.processor_config));
    if (!processor.initialize()) {
        throw std::runtime_error("failed to initialize pick processor");
    }
    std::optional<HttpApiServer> http_api_server;
    start_http_api(bootstrap, processor, http_api_server);

    catcheye::transport::WebSocketPublisher websocket(bootstrap.websocket_publisher_config);
    if (!websocket.start()) {
        throw std::runtime_error("failed to start WebSocket publisher");
    }

    std::uint64_t frame_index = 0;
    while (true) {
        catcheye::input::Frame frame;
        const auto read_status = bootstrap.camera_source->read(frame);
        if (read_status == catcheye::input::FrameReadStatus::Error) {
            throw std::runtime_error("RGB camera frame read failed");
        }
        if (read_status == catcheye::input::FrameReadStatus::EndOfStream) {
            throw std::runtime_error("RGB camera stream ended");
        }
        std::optional<catcheye::input::Frame> depth_frame;
        if (bootstrap.depth_source != nullptr) {
            catcheye::input::Frame frame;
            const auto depth_read_status = bootstrap.depth_source->read(frame);
            if (depth_read_status == catcheye::input::FrameReadStatus::Error) {
                throw std::runtime_error("depth frame read failed");
            }
            if (depth_read_status == catcheye::input::FrameReadStatus::EndOfStream) {
                throw std::runtime_error("depth stream ended");
            }
            depth_frame = std::move(frame);
        }

        PickViewerFrame viewer_frame = processor.process_viewer_frame(RgbdFrame{
            .frame_index = ++frame_index,
            .color = std::optional<catcheye::input::Frame>{std::move(frame)},
            .depth_visual = std::move(depth_frame),
        });
        const auto payloads = viewer_payload_spans(viewer_frame);
        websocket.publish_payloads(build_viewer_metadata(viewer_frame), payloads);
        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_READ_SLEEP_MS));
    }
}

int run_pick_detection(AppBootstrap bootstrap)
{
    if (bootstrap.camera_source == nullptr) {
        throw std::runtime_error("pick detection requires RGB camera input");
    }
    if (!bootstrap.camera_source->open()) {
        throw std::runtime_error("failed to open RGB camera gstreamer pipeline");
    }
    if (bootstrap.depth_source != nullptr && !bootstrap.depth_source->open()) {
        throw std::runtime_error("failed to open depth gstreamer pipeline");
    }

    PickProcessor processor(std::move(bootstrap.processor_config));
    if (!processor.initialize()) {
        throw std::runtime_error("failed to initialize pick processor");
    }
    std::optional<HttpApiServer> http_api_server;
    start_http_api(bootstrap, processor, http_api_server);

    std::optional<catcheye::transport::WebSocketPublisher> websocket;
    if (bootstrap.publisher_type == PublisherType::WebSocket) {
        websocket.emplace(bootstrap.websocket_publisher_config);
        if (!websocket->start()) {
            throw std::runtime_error("failed to start WebSocket publisher");
        }
    }

    std::uint64_t frame_index = 0;
    while (true) {
        catcheye::input::Frame frame;
        const auto read_status = bootstrap.camera_source->read(frame);
        if (read_status == catcheye::input::FrameReadStatus::Error) {
            throw std::runtime_error("RGB camera frame read failed");
        }
        if (read_status == catcheye::input::FrameReadStatus::EndOfStream) {
            throw std::runtime_error("RGB camera stream ended");
        }
        std::optional<catcheye::input::Frame> depth_frame;
        if (bootstrap.depth_source != nullptr) {
            catcheye::input::Frame frame;
            const auto depth_read_status = bootstrap.depth_source->read(frame);
            if (depth_read_status == catcheye::input::FrameReadStatus::Error) {
                throw std::runtime_error("depth frame read failed");
            }
            if (depth_read_status == catcheye::input::FrameReadStatus::EndOfStream) {
                throw std::runtime_error("depth stream ended");
            }
            depth_frame = std::move(frame);
        }

        PickDetectionFrame detection_frame = processor.process_detection_frame(RgbdFrame{
            .frame_index = ++frame_index,
            .color = std::optional<catcheye::input::Frame>{frame},
            .depth_visual = std::nullopt,
        });

        if (websocket) {
            std::vector<catcheye::Detection> detections;
            detections.reserve(detection_frame.detections.size());
            for (const auto& detection : detection_frame.detections) {
                detections.push_back(catcheye::Detection{
                    .class_id = detection.class_id,
                    .score = detection.score,
                    .box = detection.box,
                });
            }

            catcheye::input::Frame publish_frame;
            if (!catcheye::visualization::build_annotated_detection_frame(frame, detections, publish_frame)) {
                throw std::runtime_error("failed to build annotated detection frame");
            }

            PickViewerFrame viewer_frame = processor.process_viewer_frame(RgbdFrame{
                .frame_index = detection_frame.frame_index,
                .color = std::optional<catcheye::input::Frame>{std::move(publish_frame)},
                .depth_visual = std::move(depth_frame),
            });
            const auto payloads = viewer_payload_spans(viewer_frame);
            websocket->publish_payloads(build_viewer_metadata(viewer_frame, false, &detection_frame), payloads);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_READ_SLEEP_MS));
    }
}

} // namespace

AppOptions parse_app_options(int argc, char** argv)
{
    AppOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);

        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--viewer-only") {
            options.viewer_only = true;
        } else if (arg == "--camera-input") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--camera-input requires a value");
            }
            options.source_profile = parse_source_profile(argv[++i]);
        } else if (arg == "--ws") {
            if (options.publisher_type != PublisherType::None) {
                throw std::invalid_argument("only one publisher can be selected at a time");
            }
            options.publisher_type = PublisherType::WebSocket;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                options.websocket_port = std::stoi(argv[++i]);
            }
        } else if (arg == "--http-port") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--http-port requires a value");
            }
            options.http_port = std::stoi(argv[++i]);
        } else if (arg == "--camera-pipeline") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--camera-pipeline requires a value");
            }
            options.camera_pipeline = argv[++i];
        } else if (arg == "--depth-pipeline") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--depth-pipeline requires a value");
            }
            options.depth_pipeline = argv[++i];
        } else if (arg == "--roi") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--roi requires a value");
            }
            options.roi_config_path = argv[++i];
        } else if (arg == "--pallet-roi") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--pallet-roi requires a value");
            }
            options.pallet_roi_config_path = argv[++i];
        } else if (arg == "--intrinsics") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--intrinsics requires a value");
            }
            options.intrinsics_config_path = argv[++i];
        } else if (arg == "--extrinsics") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--extrinsics requires a value");
            }
            options.extrinsics_config_path = argv[++i];
        } else if (arg == "--robot-calibration") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--robot-calibration requires a value");
            }
            options.robot_calibration_config_path = argv[++i];
        } else if (arg == "--detector") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--detector requires a value");
            }
            options.detector_backend = parse_detector_backend(argv[++i]);
        } else if (arg == "--hef") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--hef requires a value");
            }
            options.hef_path = argv[++i];
        } else if (arg == "--metadata") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--metadata requires a value");
            }
            options.metadata_path = argv[++i];
        } else if (arg == "--num-threads") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--num-threads requires a value");
            }
            options.num_threads = std::stoi(argv[++i]);
        } else if (arg == "--rtsp") {
            throw std::invalid_argument("--rtsp is not supported by catcheye-pick");
        } else {
            options.positional_args.emplace_back(arg);
        }
    }

    if (options.websocket_port <= 0) {
        throw std::invalid_argument("WebSocket port must be a positive integer");
    }
    if (options.http_port <= 0) {
        throw std::invalid_argument("HTTP port must be a positive integer");
    }
    if (options.num_threads <= 0) {
        throw std::invalid_argument("--num-threads must be a positive integer");
    }
    if (options.viewer_only && options.publisher_type != PublisherType::WebSocket) {
        throw std::invalid_argument("--viewer-only requires --ws");
    }
    if (options.viewer_only && !options.positional_args.empty()) {
        throw std::invalid_argument("positional arguments are not used with --viewer-only");
    }
    if (options.viewer_only && (!options.hef_path.empty() || !options.metadata_path.empty())) {
        throw std::invalid_argument("model and metadata arguments are not used with --viewer-only");
    }

    return options;
}

AppBootstrap build_app_bootstrap(const AppOptions& options, const char* executable_path)
{
    AppBootstrap bootstrap;
    bootstrap.processor_config.detection_enabled = !options.viewer_only;
    bootstrap.processor_config.detector.backend = options.detector_backend;

    auto& ncnn_cfg = bootstrap.processor_config.detector.ncnn;
    ncnn_cfg.param_path = resolve_default_model_path(executable_path, "yolo26s_ncnn_model/model.ncnn.param");
    ncnn_cfg.bin_path = resolve_default_model_path(executable_path, "yolo26s_ncnn_model/model.ncnn.bin");
    ncnn_cfg.metadata_path = options.metadata_path.empty()
        ? resolve_default_model_path(executable_path, "yolo26s_ncnn_model/metadata.yaml")
        : options.metadata_path;
    ncnn_cfg.num_threads = options.num_threads;
    ncnn_cfg.allowed_class_ids = {39, 41, 45, 58, 63, 64, 65, 66, 67, 73, 74, 75, 76};
    if (!options.positional_args.empty()) {
        ncnn_cfg.param_path = options.positional_args[0];
    }
    if (options.positional_args.size() > 1) {
        ncnn_cfg.bin_path = options.positional_args[1];
    }
    if (options.positional_args.size() > 2 && options.metadata_path.empty()) {
        ncnn_cfg.metadata_path = options.positional_args[2];
    }

    auto& hailo_cfg = bootstrap.processor_config.detector.hailo;
    hailo_cfg.hef_path = options.hef_path;
    hailo_cfg.metadata_path = options.metadata_path.empty()
        ? resolve_default_model_path(executable_path, "yolo26s_ncnn_model/metadata.yaml")
        : options.metadata_path;
    hailo_cfg.allowed_class_ids = {39, 41, 45, 58, 63, 64, 65, 66, 67, 73, 74, 75, 76};

    bootstrap.processor_config.roi_config_path = options.roi_config_path.empty()
        ? resolve_default_config_path(executable_path, "roi_cam_default.json")
        : options.roi_config_path;
    bootstrap.processor_config.pallet_roi_config_path = options.pallet_roi_config_path.empty()
        ? resolve_default_config_path(executable_path, "pallet_roi_cam_default.json")
        : options.pallet_roi_config_path;
    bootstrap.processor_config.roi_config = load_roi_config(bootstrap.processor_config.roi_config_path);
    bootstrap.processor_config.roi_enabled = true;
    bootstrap.processor_config.pallet_roi_config = load_roi_config(bootstrap.processor_config.pallet_roi_config_path);
    bootstrap.processor_config.pallet_roi_enabled = true;

    bootstrap.intrinsics_config_path = options.intrinsics_config_path.empty()
        ? resolve_default_config_path(executable_path, "intrinsics.json")
        : options.intrinsics_config_path;
    bootstrap.extrinsics_config_path = options.extrinsics_config_path.empty()
        ? resolve_default_config_path(executable_path, "extrinsics.json")
        : options.extrinsics_config_path;
    bootstrap.robot_calibration_config_path = options.robot_calibration_config_path.empty()
        ? resolve_default_config_path(executable_path, "robot_calibration.json")
        : options.robot_calibration_config_path;
    bootstrap.processor_config.robot_calibration_config_path = bootstrap.robot_calibration_config_path;
    bootstrap.processor_config.robot_calibration = load_robot_calibration_config(bootstrap.robot_calibration_config_path);

    bootstrap.publisher_type = options.publisher_type;
    bootstrap.websocket_publisher_config.port = options.websocket_port;
    bootstrap.http_api_server_config.port = options.http_port;

    const std::string camera_pipeline = options.camera_pipeline.empty() ? std::string(DEFAULT_CAMERA_PIPELINE) : options.camera_pipeline;
    bootstrap.camera_source = catcheye::input::create_frame_source(catcheye::input::InputSourceConfig{
        .type = catcheye::input::InputSourceType::Camera,
        .uri = {},
        .camera_pipeline = camera_pipeline,
        .camera_device = {},
        .camera_width = 1280,
        .camera_height = 720,
    });
    if (!options.depth_pipeline.empty()) {
        bootstrap.depth_source = catcheye::input::create_frame_source(catcheye::input::InputSourceConfig{
            .type = catcheye::input::InputSourceType::Camera,
            .uri = {},
            .camera_pipeline = options.depth_pipeline,
            .camera_device = {},
            .camera_width = 1280,
            .camera_height = 720,
        });
    }

    if (!options.viewer_only && options.detector_backend == catcheye::DetectorBackend::Ncnn &&
        (ncnn_cfg.param_path.empty() || ncnn_cfg.bin_path.empty())) {
        throw std::runtime_error("NCNN model paths are required");
    }
    if (!options.viewer_only && options.detector_backend == catcheye::DetectorBackend::Hailo && hailo_cfg.hef_path.empty()) {
        throw std::runtime_error("Hailo HEF path is required; pass --hef <model.hef>");
    }

    return bootstrap;
}

int run_app(int argc, char** argv)
{
    const AppOptions options = parse_app_options(argc, argv);

    if (options.show_help) {
        print_usage();
        return 0;
    }

    AppBootstrap bootstrap = build_app_bootstrap(options, argv[0]);
    std::cerr << "catcheye-pick starting (mode='" << describe_runtime_mode(options) << "', publisher='"
              << publisher_name(bootstrap.publisher_type) << "'";
    if (!options.viewer_only) {
        std::cerr << ", detector='" << detector_backend_name(options.detector_backend) << "'";
    }
    std::cerr << ")\n";

    if (options.viewer_only) {
        return run_viewer_only(std::move(bootstrap));
    }
    return run_pick_detection(std::move(bootstrap));
}

} // namespace catcheye::pick
