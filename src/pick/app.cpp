#include "pick/app.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
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
#include "pick/config_json.hpp"
#include "pick/http_api_server.hpp"
#include "pick/processor.hpp"
#include "pick/robot_calibration_repository.hpp"
#include "pick/viewer_metadata.hpp"

namespace catcheye::pick {
namespace {

constexpr int CAMERA_READ_SLEEP_MS = 1;

void print_usage()
{
    std::cout << "Usage:\n"
              << "  catcheye-pick [options]\n"
              << "\n"
              << "Options:\n"
              << "  -h, --help                  Show this help\n"
              << "  --input-source <kind>       Input source: camera | image | video (default: camera; image/video not implemented)\n"
              << "  --camera-backend <name>     Camera backend: isaacsim | realsense (default: isaacsim; realsense not implemented)\n"
              << "  --camera-pipeline <pipe>    Isaac Sim color GStreamer pipeline; required for isaacsim\n"
              << "  --depth-pipeline <pipe>     Isaac Sim depth visualization GStreamer pipeline\n"
              << "  --depth-max-m <meters>      Max metric depth represented by depth frame brightness; required for detection with depth\n"
              << "  --depth-min-m <meters>      Min valid metric depth (default: 0.05)\n"
              << "  --viewer-only               Disable detection; requires --ws\n"
              << "  --ws [port]                 Publish frames over WebSocket (default port: 8080)\n"
              << "  --http-port <port>          HTTP API port (default: 8090)\n"
              << "  --detector <name>           Detector backend: hailo (default: hailo)\n"
              << "  --hef <path>                Hailo HEF model path\n"
              << "  --metadata <path>           Detector metadata YAML path\n"
              << "  --roi <path>                Person ROI config path (default: config/roi_cam_default.json)\n"
              << "  --pallet-roi <path>         Pallet ROI config path (default: config/pallet_roi_cam_default.json)\n"
              << "  --intrinsics <path>         Camera intrinsics JSON path (default: config/intrinsics.json)\n"
              << "  --extrinsics <path>         Camera extrinsics JSON path (default: config/extrinsics.json)\n"
              << "  --robot-calibration <path>  Robot calibration config path (default: config/robot_calibration.json)\n"
              << "  --objects <path>            Object catalog JSON path (default: config/objects.json)\n"
              << "\n"
              << "Examples:\n"
              << "  catcheye-pick --help\n"
              << "  catcheye-pick --ws --viewer-only --camera-pipeline \"<gst-color-pipeline>\"\n"
              << "  catcheye-pick --ws --detector hailo --hef models/yolo26m_hailo_model/yolo26m.hef --camera-pipeline \"<gst-color-pipeline>\" --depth-pipeline \"<gst-depth-pipeline>\" --depth-max-m 5.0\n";
}

catcheye::DetectorBackend parse_detector_backend(std::string_view value)
{
    if (value == "hailo") {
        return catcheye::DetectorBackend::Hailo;
    }
    throw std::invalid_argument("unknown detector backend: " + std::string(value));
}

InputSourceKind parse_input_source(std::string_view value)
{
    if (value == "camera") {
        return InputSourceKind::Camera;
    }
    if (value == "image") {
        return InputSourceKind::Image;
    }
    if (value == "video") {
        return InputSourceKind::Video;
    }
    throw std::invalid_argument("unknown input source: " + std::string(value));
}

CameraBackend parse_camera_backend(std::string_view value)
{
    if (value == "realsense") {
        return CameraBackend::Realsense;
    }
    if (value == "isaacsim") {
        return CameraBackend::IsaacSim;
    }
    throw std::invalid_argument("unknown camera backend: " + std::string(value));
}

std::string_view read_required_value(std::span<char* const> args, std::size_t& index, std::string_view flag)
{
    if (index + 1 >= args.size()) {
        throw std::invalid_argument(std::string(flag) + " requires a value");
    }
    return args[++index];
}

const char* input_source_name(InputSourceKind kind)
{
    switch (kind) {
    case InputSourceKind::Camera:
        return "camera";
    case InputSourceKind::Image:
        return "image";
    case InputSourceKind::Video:
        return "video";
    }
    return "unknown";
}

const char* camera_backend_name(CameraBackend backend)
{
    switch (backend) {
    case CameraBackend::Realsense:
        return "realsense";
    case CameraBackend::IsaacSim:
        return "isaacsim";
    }
    return "unknown";
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

CameraIntrinsicsConfig load_camera_intrinsics_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("failed to load camera intrinsics config: " + path);
    }
    const std::string body((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    CameraIntrinsicsConfig intrinsics;
    if (!parse_json_int_field(body, "width", intrinsics.width) || !parse_json_int_field(body, "height", intrinsics.height) ||
        !parse_json_float_field(body, "fx", intrinsics.fx) || !parse_json_float_field(body, "fy", intrinsics.fy) ||
        !parse_json_float_field(body, "cx", intrinsics.cx) || !parse_json_float_field(body, "cy", intrinsics.cy)) {
        throw std::runtime_error("camera intrinsics config is missing width/height/fx/fy/cx/cy: " + path);
    }
    if (intrinsics.width <= 0 || intrinsics.height <= 0 || intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F) {
        throw std::runtime_error("camera intrinsics config has invalid values: " + path);
    }
    return intrinsics;
}

std::vector<std::string_view> json_object_blocks(std::string_view body, std::string_view array_key)
{
    const std::string quoted_key = "\"" + std::string(array_key) + "\"";
    const std::size_t key_pos = body.find(quoted_key);
    if (key_pos == std::string_view::npos) {
        return {};
    }
    const std::size_t array_open = body.find('[', key_pos + quoted_key.size());
    if (array_open == std::string_view::npos) {
        return {};
    }

    std::vector<std::string_view> blocks;
    int depth = 0;
    std::size_t object_start = std::string_view::npos;
    for (std::size_t i = array_open + 1U; i < body.size(); ++i) {
        const char ch = body[i];
        if (ch == '{') {
            if (depth == 0) {
                object_start = i;
            }
            ++depth;
        } else if (ch == '}') {
            --depth;
            if (depth == 0 && object_start != std::string_view::npos) {
                blocks.push_back(body.substr(object_start, i - object_start + 1U));
                object_start = std::string_view::npos;
            }
        } else if (ch == ']' && depth == 0) {
            break;
        }
    }
    return blocks;
}

std::vector<ProductObjectConfig> load_object_catalog_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("failed to load object catalog config: " + path);
    }
    const std::string body((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const auto blocks = json_object_blocks(body, "objects");
    if (blocks.empty()) {
        throw std::runtime_error("object catalog has no objects: " + path);
    }

    std::vector<ProductObjectConfig> objects;
    objects.reserve(blocks.size());
    for (const std::string_view block : blocks) {
        ProductObjectConfig object;
        if (!parse_json_string_field(block, "product_id", object.product_id) || object.product_id.empty()) {
            throw std::runtime_error("object catalog entry is missing product_id: " + path);
        }
        std::vector<float> pick_point;
        if (parse_json_float_array_field(block, "pick_point_object_m", pick_point)) {
            if (pick_point.size() != 3U) {
                throw std::runtime_error("pick_point_object_m must have 3 values: " + path);
            }
            object.pick_point_object_m = ObjectPointConfig{
                .x = pick_point[0],
                .y = pick_point[1],
                .z = pick_point[2],
            };
        }
        objects.push_back(std::move(object));
    }
    return objects;
}

std::string describe_runtime_mode(const AppOptions& options)
{
    const char* processing_name = options.viewer_only ? "viewer only" : "pick detection";
    const char* output_name = options.publisher_type == PublisherType::WebSocket ? "websocket output" : "local output";
    return std::string(input_source_name(options.input_source)) + "/" + camera_backend_name(options.camera_backend) + " + " +
        processing_name + " + " + output_name;
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
        bootstrap.object_catalog_config_path,
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
            .depth_visual = depth_frame,
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

    const std::span<char* const> args(argv, static_cast<std::size_t>(argc));
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view arg(args[i]);

        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--viewer-only") {
            options.viewer_only = true;
        } else if (arg == "--input-source") {
            options.input_source = parse_input_source(read_required_value(args, i, arg));
        } else if (arg == "--camera-backend") {
            options.camera_backend = parse_camera_backend(read_required_value(args, i, arg));
        } else if (arg == "--ws") {
            if (options.publisher_type != PublisherType::None) {
                throw std::invalid_argument("only one publisher can be selected at a time");
            }
            options.publisher_type = PublisherType::WebSocket;
            if (i + 1 < args.size() && args[i + 1][0] != '-') {
                options.websocket_port = std::stoi(std::string(read_required_value(args, i, arg)));
            }
        } else if (arg == "--http-port") {
            options.http_port = std::stoi(std::string(read_required_value(args, i, arg)));
        } else if (arg == "--camera-pipeline") {
            options.camera_pipeline = read_required_value(args, i, arg);
        } else if (arg == "--depth-pipeline") {
            options.depth_pipeline = read_required_value(args, i, arg);
        } else if (arg == "--depth-max-m") {
            options.depth_max_m = std::stof(std::string(read_required_value(args, i, arg)));
        } else if (arg == "--depth-min-m") {
            options.depth_min_m = std::stof(std::string(read_required_value(args, i, arg)));
        } else if (arg == "--roi") {
            options.roi_config_path = read_required_value(args, i, arg);
        } else if (arg == "--pallet-roi") {
            options.pallet_roi_config_path = read_required_value(args, i, arg);
        } else if (arg == "--intrinsics") {
            options.intrinsics_config_path = read_required_value(args, i, arg);
        } else if (arg == "--extrinsics") {
            options.extrinsics_config_path = read_required_value(args, i, arg);
        } else if (arg == "--robot-calibration") {
            options.robot_calibration_config_path = read_required_value(args, i, arg);
        } else if (arg == "--objects") {
            options.object_catalog_config_path = read_required_value(args, i, arg);
        } else if (arg == "--detector") {
            options.detector_backend = parse_detector_backend(read_required_value(args, i, arg));
        } else if (arg == "--hef") {
            options.hef_path = read_required_value(args, i, arg);
        } else if (arg == "--metadata") {
            options.metadata_path = read_required_value(args, i, arg);
        } else if (!arg.empty() && arg.front() == '-') {
            throw std::invalid_argument("unknown option: " + std::string(arg));
        } else {
            throw std::invalid_argument("unexpected positional argument: " + std::string(arg));
        }
    }

    if (options.show_help) {
        return options;
    }
    if (options.websocket_port <= 0) {
        throw std::invalid_argument("WebSocket port must be a positive integer");
    }
    if (options.http_port <= 0) {
        throw std::invalid_argument("HTTP port must be a positive integer");
    }
    if (options.viewer_only && options.publisher_type != PublisherType::WebSocket) {
        throw std::invalid_argument("--viewer-only requires --ws");
    }
    if (options.viewer_only && (!options.hef_path.empty() || !options.metadata_path.empty())) {
        throw std::invalid_argument("model and metadata arguments are not used with --viewer-only");
    }
    if (options.input_source != InputSourceKind::Camera) {
        throw std::invalid_argument("--input-source image/video is not implemented yet");
    }
    if (options.camera_backend != CameraBackend::IsaacSim && (!options.camera_pipeline.empty() || !options.depth_pipeline.empty())) {
        throw std::invalid_argument("--camera-pipeline and --depth-pipeline are only used with --camera-backend isaacsim");
    }
    if (options.depth_max_m.has_value() && options.depth_max_m.value() <= 0.0F) {
        throw std::invalid_argument("--depth-max-m must be greater than zero");
    }
    if (options.depth_min_m < 0.0F) {
        throw std::invalid_argument("--depth-min-m must be zero or greater");
    }
    if (options.depth_max_m.has_value() && options.depth_min_m >= options.depth_max_m.value()) {
        throw std::invalid_argument("--depth-min-m must be less than --depth-max-m");
    }
    if (options.depth_max_m.has_value() && options.depth_pipeline.empty()) {
        throw std::invalid_argument("--depth-max-m requires --depth-pipeline");
    }
    if (!options.viewer_only && !options.depth_pipeline.empty() && !options.depth_max_m.has_value()) {
        throw std::invalid_argument("pick detection with --depth-pipeline requires --depth-max-m");
    }
    if (options.camera_backend != CameraBackend::IsaacSim) {
        throw std::invalid_argument("--camera-backend realsense is not implemented yet");
    }
    if (options.camera_pipeline.empty()) {
        throw std::invalid_argument("--camera-backend isaacsim requires --camera-pipeline");
    }

    return options;
}

AppBootstrap build_app_bootstrap(const AppOptions& options, const char* executable_path)
{
    AppBootstrap bootstrap;
    bootstrap.processor_config.detection_enabled = !options.viewer_only;
    bootstrap.processor_config.detector.backend = options.detector_backend;

    auto& hailo_cfg = bootstrap.processor_config.detector.hailo;
    hailo_cfg.hef_path = options.hef_path;
    hailo_cfg.metadata_path = options.metadata_path.empty()
        ? resolve_default_model_path(executable_path, "yolo26m_hailo_model/metadata.yaml")
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
    bootstrap.object_catalog_config_path = options.object_catalog_config_path.empty()
        ? resolve_default_config_path(executable_path, "objects.json")
        : options.object_catalog_config_path;
    bootstrap.processor_config.robot_calibration_config_path = bootstrap.robot_calibration_config_path;
    bootstrap.processor_config.robot_calibration = load_robot_calibration_config(bootstrap.robot_calibration_config_path);
    bootstrap.processor_config.camera_intrinsics = load_camera_intrinsics_config(bootstrap.intrinsics_config_path);
    bootstrap.processor_config.object_catalog = load_object_catalog_config(bootstrap.object_catalog_config_path);
    if (options.depth_max_m.has_value()) {
        bootstrap.processor_config.depth_projection = DepthProjectionConfig{
            .enabled = true,
            .min_depth_m = options.depth_min_m,
            .max_depth_m = options.depth_max_m.value(),
        };
    }

    bootstrap.publisher_type = options.publisher_type;
    bootstrap.websocket_publisher_config.port = options.websocket_port;
    bootstrap.http_api_server_config.port = options.http_port;

    bootstrap.camera_source = catcheye::input::create_frame_source(catcheye::input::InputSourceConfig{
        .type = catcheye::input::InputSourceType::Camera,
        .uri = {},
        .camera_pipeline = options.camera_pipeline,
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
