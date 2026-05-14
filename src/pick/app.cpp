#include "pick/app.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "CubeEyeCamera.h"
#include "CubeEyeSource.h"
#include "catcheye/input/frame_source.hpp"
#include "catcheye/roi/roi_repository.hpp"
#include "catcheye/roi/roi_validation.hpp"
#include "catcheye/transport/websocket_publisher.hpp"
#include "catcheye/visualization/annotation_renderer.hpp"
#include "pick/cubeeye_camera.hpp"
#include "pick/http_api_server.hpp"
#include "pick/processor.hpp"
#include "pick/viewer_metadata.hpp"

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
              << "  --list-cubeeye             List connected CubeEye camera sources\n"
              << "  --detector <name>         Detector backend: ncnn | hailo\n"
              << "  --hef <path>              Hailo HEF model path\n"
              << "  --metadata <path>         Detector metadata YAML path\n"
              << "  --num-threads <count>     NCNN inference threads (default: 2)\n"
              << "  --camera-input <mode>     Camera input: rgb-cubeeye | rgb | cubeeye\n"
              << "  --viewer-only             Start selected camera input without detection\n"
              << "  --ws [port]               Publish frames over WebSocket\n"
              << "  --http-port <port>        HTTP API port (default: 8090)\n"
              << "  --camera-pipeline <pipe>  GStreamer pipeline for Camera Module 3\n"
              << "  --roi <path>              Person ROI config path\n"
              << "  --pallet-roi <path>       Pallet ROI config path\n"
              << "  --cubeeye-frames <list>    CubeEye frames: depth, amplitude, rgb, pointcloud (depth and pointcloud are exclusive)\n"
              << "  --cubeeye-camera-fps <fps>  CubeEye S111D camera framerate: 7 | 15 | 30\n"
              << "  --rgb-cubeeye-offset-u <value>  RGB to CubeEye normalized U offset (default: 0.00)\n"
              << "  --rgb-cubeeye-offset-v <value>  RGB to CubeEye normalized V offset (default: 0.40)\n"
              << "  --pointcloud-downsample <stride>  PointCloud downsample stride (default: 4)\n";
}

catcheye::DetectorBackend parse_detector_backend(std::string_view value) {
    if (value == "ncnn") {
        return catcheye::DetectorBackend::Ncnn;
    }
    if (value == "hailo") {
        return catcheye::DetectorBackend::Hailo;
    }

    throw std::invalid_argument("unknown detector backend: " + std::string(value));
}

CameraInputMode parse_camera_input_mode(std::string_view value) {
    if (value == "rgb-cubeeye") {
        return CameraInputMode::RgbCubeEye;
    }
    if (value == "rgb") {
        return CameraInputMode::RgbOnly;
    }
    if (value == "cubeeye") {
        return CameraInputMode::CubeEyeOnly;
    }

    throw std::invalid_argument("unknown camera input mode: " + std::string(value));
}

const char* camera_input_mode_name(CameraInputMode mode) {
    switch (mode) {
    case CameraInputMode::RgbCubeEye:
        return "RGB camera + CubeEye";
    case CameraInputMode::RgbOnly:
        return "RGB camera";
    case CameraInputMode::CubeEyeOnly:
        return "CubeEye";
    }
    return "unknown";
}

bool uses_rgb_camera(CameraInputMode mode) {
    return mode == CameraInputMode::RgbCubeEye || mode == CameraInputMode::RgbOnly;
}

bool uses_cubeeye(CameraInputMode mode) {
    return mode == CameraInputMode::RgbCubeEye || mode == CameraInputMode::CubeEyeOnly;
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

bool is_supported_cubeeye_camera_fps(int fps)
{
    return fps == 7 || fps == 15 || fps == 30;
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

class LatestViewerFramePublisher {
private:
    struct ViewerPacket {
        std::string source_key;
        std::string metadata;
        PickViewerFrame frame;
    };

public:
    explicit LatestViewerFramePublisher(catcheye::transport::WebSocketPublisher& websocket)
        : websocket_(websocket)
        , thread_(&LatestViewerFramePublisher::run, this)
    {}

    ~LatestViewerFramePublisher()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_one();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void publish(std::string source_key, PickViewerFrame frame)
    {
        publish_with_metadata(std::move(source_key), build_viewer_metadata(frame), std::move(frame));
    }

    void publish_with_metadata(std::string source_key, std::string metadata, PickViewerFrame frame)
    {
        throw_if_failed();
        ViewerPacket packet{
            .source_key = std::move(source_key),
            .metadata = std::move(metadata),
            .frame = std::move(frame),
        };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto existing = std::find_if(pending_packets_.begin(), pending_packets_.end(), [&](const ViewerPacket& pending) {
                return pending.source_key == packet.source_key;
            });
            if (existing == pending_packets_.end()) {
                pending_packets_.push_back(std::move(packet));
            } else {
                *existing = std::move(packet);
            }
        }
        condition_.notify_one();
    }

    void throw_if_failed()
    {
        std::exception_ptr error;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            error = error_;
        }
        if (error) {
            std::rethrow_exception(error);
        }
    }

private:
    void run()
    {
        try {
            while (true) {
                ViewerPacket packet;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [&] {
                        return stopping_ || !pending_packets_.empty();
                    });
                    if (stopping_ && pending_packets_.empty()) {
                        return;
                    }
                    packet = std::move(pending_packets_.front());
                    pending_packets_.erase(pending_packets_.begin());
                }

                const auto payloads = viewer_payload_spans(packet.frame);
                websocket_.publish_payloads(packet.metadata, payloads);
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = std::current_exception();
        }
    }

    catcheye::transport::WebSocketPublisher& websocket_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<ViewerPacket> pending_packets_;
    bool stopping_{false};
    std::exception_ptr error_;
};

class LatestCubeEyeFrameReader {
public:
    explicit LatestCubeEyeFrameReader(CubeEyeCameraSession& cubeeye)
        : cubeeye_(cubeeye)
        , thread_(&LatestCubeEyeFrameReader::run, this)
    {}

    ~LatestCubeEyeFrameReader()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    CubeEyeFrameSet latest()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] {
            return latest_.has_value() || error_ || stopping_;
        });
        if (error_) {
            std::rethrow_exception(error_);
        }
        if (!latest_) {
            throw std::runtime_error("CubeEye frame reader stopped before receiving frames");
        }
        return *latest_;
    }

    CubeEyeFrameSet latest_after(std::uint64_t last_sequence)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] {
            return (latest_.has_value() && latest_->sequence != last_sequence) || error_ || stopping_;
        });
        if (error_) {
            std::rethrow_exception(error_);
        }
        if (!latest_ || latest_->sequence == last_sequence) {
            throw std::runtime_error("CubeEye frame reader stopped before receiving frames");
        }
        return *latest_;
    }

private:
    void run()
    {
        try {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        return;
                    }
                }

                CubeEyeFrameSet frame_set = cubeeye_.read();

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        return;
                    }
                    latest_ = std::move(frame_set);
                }
                condition_.notify_all();
            }
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = std::current_exception();
            }
            condition_.notify_all();
        }
    }

    CubeEyeCameraSession& cubeeye_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<CubeEyeFrameSet> latest_;
    bool stopping_{false};
    std::exception_ptr error_;
};

class LatestRgbFrameReader {
public:
    struct FrameSnapshot {
        std::uint64_t sequence = 0;
        catcheye::input::Frame frame;
    };

    explicit LatestRgbFrameReader(catcheye::input::FrameSource& camera_source)
        : camera_source_(camera_source)
        , thread_(&LatestRgbFrameReader::run, this)
    {}

    ~LatestRgbFrameReader()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    FrameSnapshot latest_after(std::uint64_t last_sequence)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] {
            return (latest_.has_value() && latest_->sequence != last_sequence) || error_ || stopping_;
        });
        if (error_) {
            std::rethrow_exception(error_);
        }
        if (!latest_ || latest_->sequence == last_sequence) {
            throw std::runtime_error("RGB frame reader stopped before receiving frames");
        }
        return *latest_;
    }

private:
    void run()
    {
        try {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        return;
                    }
                }

                catcheye::input::Frame frame;
                const auto read_status = camera_source_.read(frame);
                if (read_status == catcheye::input::FrameReadStatus::Error) {
                    throw std::runtime_error("Camera Module 3 frame read failed");
                }
                if (read_status == catcheye::input::FrameReadStatus::EndOfStream) {
                    throw std::runtime_error("Camera Module 3 stream ended");
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        return;
                    }
                    latest_ = FrameSnapshot{
                        .sequence = ++sequence_,
                        .frame = std::move(frame),
                    };
                }
                condition_.notify_all();
                std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_READ_SLEEP_MS));
            }
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = std::current_exception();
            }
            condition_.notify_all();
        }
    }

    catcheye::input::FrameSource& camera_source_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<FrameSnapshot> latest_;
    std::uint64_t sequence_ = 0;
    bool stopping_{false};
    std::exception_ptr error_;
};

std::exception_ptr worker_error_snapshot(std::mutex& error_mutex, const std::exception_ptr& worker_error)
{
    std::lock_guard<std::mutex> lock(error_mutex);
    return worker_error;
}

std::string describe_runtime_mode(const AppOptions& options) {
    const char* processing_name = options.viewer_only ? "viewer only" : "pick detection";
    const char* output_name = options.publisher_type == PublisherType::WebSocket ? "websocket output" : "local output";
    return std::string(camera_input_mode_name(options.camera_input_mode)) + " + " + processing_name + " + " + output_name;
}

int run_viewer_only(AppBootstrap bootstrap) {
    const bool rgb_camera_enabled = bootstrap.camera_source != nullptr;
    const bool cubeeye_enabled = !bootstrap.processor_config.cubeeye_frames.empty();

    std::optional<CubeEyeCameraSession> cubeeye;
    if (cubeeye_enabled) {
        cubeeye.emplace(bootstrap.processor_config.cubeeye_frames, bootstrap.cubeeye_camera_fps);
        cubeeye->open();
    }

    if (rgb_camera_enabled && !bootstrap.camera_source->open()) {
        throw std::runtime_error("failed to open Camera Module 3 gstreamer pipeline");
    }

    const std::string http_roi_config_path = bootstrap.processor_config.roi_config_path;
    const std::string http_pallet_roi_config_path = bootstrap.processor_config.pallet_roi_config_path;
    PickProcessor processor(std::move(bootstrap.processor_config));
    if (!processor.initialize()) {
        throw std::runtime_error("failed to initialize pick processor");
    }
    HttpApiServer http_api_server(
        bootstrap.http_api_server_config,
        http_roi_config_path,
        http_pallet_roi_config_path,
        &processor,
        cubeeye ? &*cubeeye : nullptr);
    if (!http_api_server.start()) {
        throw std::runtime_error("failed to start HTTP API server");
    }

    catcheye::transport::WebSocketPublisher websocket(bootstrap.websocket_publisher_config);
    if (!websocket.start()) {
        throw std::runtime_error("failed to start WebSocket publisher");
    }
    LatestViewerFramePublisher async_publisher(websocket);

    std::atomic_bool running{true};
    std::atomic_uint64_t frame_index{0};
    std::mutex worker_error_mutex;
    std::exception_ptr worker_error;
    auto capture_worker_error = [&] {
        {
            std::lock_guard<std::mutex> lock(worker_error_mutex);
            if (!worker_error) {
                worker_error = std::current_exception();
            }
        }
        running = false;
    };

    std::vector<std::thread> workers;
    if (rgb_camera_enabled) {
        workers.emplace_back([&] {
            try {
                while (running) {
                    catcheye::input::Frame frame;
                    const auto read_status = bootstrap.camera_source->read(frame);
                    if (read_status == catcheye::input::FrameReadStatus::Error) {
                        throw std::runtime_error("Camera Module 3 frame read failed");
                    }
                    if (read_status == catcheye::input::FrameReadStatus::EndOfStream) {
                        throw std::runtime_error("Camera Module 3 stream ended");
                    }

                    PickViewerFrame viewer_frame =
                        processor.process_viewer_frame(std::optional<catcheye::input::Frame>{std::move(frame)}, {}, ++frame_index);
                    async_publisher.publish("camera", std::move(viewer_frame));
                    std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_READ_SLEEP_MS));
                }
            } catch (...) {
                capture_worker_error();
            }
        });
    }

    if (cubeeye_enabled) {
        workers.emplace_back([&] {
            try {
                while (running) {
                    CubeEyeFrameSet cubeeye_frames = cubeeye->read();
                    PickViewerFrame viewer_frame = processor.process_viewer_frame(std::nullopt, cubeeye_frames, ++frame_index);
                    async_publisher.publish("cubeeye", std::move(viewer_frame));
                }
            } catch (...) {
                capture_worker_error();
            }
        });
    }

    while (running) {
        try {
            async_publisher.throw_if_failed();
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(worker_error_mutex);
                if (!worker_error) {
                    worker_error = std::current_exception();
                }
            }
            running = false;
            break;
        }
        if (worker_error_snapshot(worker_error_mutex, worker_error)) {
            running = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    async_publisher.throw_if_failed();
    if (const auto error = worker_error_snapshot(worker_error_mutex, worker_error)) {
        std::rethrow_exception(error);
    }

    return 0;
}

int run_pick_detection(AppBootstrap bootstrap)
{
    if (bootstrap.camera_source == nullptr) {
        throw std::runtime_error("pick detection requires RGB camera input");
    }

    const std::string http_roi_config_path = bootstrap.processor_config.roi_config_path;
    const std::string http_pallet_roi_config_path = bootstrap.processor_config.pallet_roi_config_path;
    std::vector<CubeEyeFrameSpec> cubeeye_frame_specs = bootstrap.processor_config.cubeeye_frames;
    const int cubeeye_camera_fps = bootstrap.cubeeye_camera_fps;
    PickProcessor processor(std::move(bootstrap.processor_config));

    std::optional<CubeEyeCameraSession> cubeeye;
    if (!cubeeye_frame_specs.empty()) {
        cubeeye.emplace(std::move(cubeeye_frame_specs), cubeeye_camera_fps);
        cubeeye->open();
    }
    std::optional<LatestCubeEyeFrameReader> cubeeye_reader;
    if (cubeeye) {
        cubeeye_reader.emplace(*cubeeye);
    }

    if (!bootstrap.camera_source->open()) {
        throw std::runtime_error("failed to open Camera Module 3 gstreamer pipeline");
    }

    if (!processor.initialize()) {
        throw std::runtime_error("failed to initialize pick processor");
    }

    HttpApiServer http_api_server(
        bootstrap.http_api_server_config,
        http_roi_config_path,
        http_pallet_roi_config_path,
        &processor,
        cubeeye ? &*cubeeye : nullptr);
    if (!http_api_server.start()) {
        throw std::runtime_error("failed to start HTTP API server");
    }

    std::optional<catcheye::transport::WebSocketPublisher> websocket;
    std::optional<LatestViewerFramePublisher> async_publisher;
    if (bootstrap.publisher_type == PublisherType::WebSocket) {
        websocket.emplace(bootstrap.websocket_publisher_config);
        if (!websocket->start()) {
            throw std::runtime_error("failed to start WebSocket publisher");
        }
        async_publisher.emplace(*websocket);
    }

    LatestRgbFrameReader rgb_reader(*bootstrap.camera_source);
    std::atomic_bool running{true};
    std::mutex worker_error_mutex;
    std::exception_ptr worker_error;
    std::mutex latest_detection_mutex;
    std::optional<PickDetectionFrame> latest_detection_frame;
    auto capture_worker_error = [&] {
        {
            std::lock_guard<std::mutex> lock(worker_error_mutex);
            if (!worker_error) {
                worker_error = std::current_exception();
            }
        }
        running = false;
    };

    std::optional<std::thread> cubeeye_publish_worker;
    if (async_publisher && cubeeye_reader) {
        cubeeye_publish_worker.emplace([&] {
            try {
                std::uint64_t published_cubeeye_sequence = 0;
                while (running) {
                    CubeEyeFrameSet cubeeye_frames = cubeeye_reader->latest_after(published_cubeeye_sequence);
                    PickViewerFrame cubeeye_viewer_frame = processor.process_viewer_frame(std::nullopt, cubeeye_frames, cubeeye_frames.sequence);
                    std::optional<PickDetectionFrame> detection_snapshot;
                    {
                        std::lock_guard<std::mutex> lock(latest_detection_mutex);
                        detection_snapshot = latest_detection_frame;
                    }
                    const std::string cubeeye_metadata = build_viewer_metadata(
                        cubeeye_viewer_frame,
                        false,
                        detection_snapshot ? &*detection_snapshot : nullptr);
                    async_publisher->publish_with_metadata("cubeeye", std::move(cubeeye_metadata), std::move(cubeeye_viewer_frame));
                    published_cubeeye_sequence = cubeeye_frames.sequence;
                }
            } catch (...) {
                capture_worker_error();
            }
        });
    }

    try {
        std::uint64_t frame_index = 0;
        std::uint64_t last_rgb_sequence = 0;
        while (running) {
            if (async_publisher) {
                async_publisher->throw_if_failed();
            }
            if (worker_error_snapshot(worker_error_mutex, worker_error)) {
                break;
            }

            LatestRgbFrameReader::FrameSnapshot rgb_snapshot = rgb_reader.latest_after(last_rgb_sequence);
            last_rgb_sequence = rgb_snapshot.sequence;

            CubeEyeFrameSet cubeeye_frames;
            if (cubeeye_reader) {
                cubeeye_frames = cubeeye_reader->latest();
            }

            PickDetectionFrame detection_frame = processor.process_detection_frame(rgb_snapshot.frame, cubeeye_frames, ++frame_index);
            {
                std::lock_guard<std::mutex> lock(latest_detection_mutex);
                latest_detection_frame = detection_frame;
            }
            if (async_publisher) {
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
                if (!catcheye::visualization::build_annotated_detection_frame(rgb_snapshot.frame, detections, publish_frame)) {
                    throw std::runtime_error("failed to build annotated detection frame");
                }

                PickViewerFrame viewer_frame = processor.process_viewer_frame(
                    std::optional<catcheye::input::Frame>{std::move(publish_frame)},
                    {},
                    detection_frame.frame_index);
                const std::string metadata = build_viewer_metadata(viewer_frame, false, &detection_frame);
                async_publisher->publish_with_metadata("camera", std::move(metadata), std::move(viewer_frame));
            }
        }
    } catch (...) {
        running = false;
        if (cubeeye_publish_worker && cubeeye_publish_worker->joinable()) {
            cubeeye_publish_worker->join();
        }
        throw;
    }

    running = false;
    if (cubeeye_publish_worker && cubeeye_publish_worker->joinable()) {
        cubeeye_publish_worker->join();
    }
    if (const auto error = worker_error_snapshot(worker_error_mutex, worker_error)) {
        std::rethrow_exception(error);
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
        } else if (arg == "--list-cubeeye") {
            options.list_cubeeye_sources = true;
        } else if (arg == "--viewer-only") {
            options.viewer_only = true;
        } else if (arg == "--camera-input") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--camera-input requires a value");
            }
            options.camera_input_mode = parse_camera_input_mode(argv[++i]);
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
            options.camera_pipeline_set = true;
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
        } else if (arg == "--cubeeye-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--cubeeye-frames requires a value");
            }
            options.cubeeye_frames = argv[++i];
            options.cubeeye_frames_set = true;
            parse_cubeeye_frames(options.cubeeye_frames);
        } else if (arg == "--pointcloud-downsample") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--pointcloud-downsample requires a value");
            }
            options.pointcloud_downsample = std::stoi(argv[++i]);
        } else if (arg == "--rgb-cubeeye-offset-u") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--rgb-cubeeye-offset-u requires a value");
            }
            options.rgb_cubeeye_offset_u = std::stof(argv[++i]);
        } else if (arg == "--rgb-cubeeye-offset-v") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--rgb-cubeeye-offset-v requires a value");
            }
            options.rgb_cubeeye_offset_v = std::stof(argv[++i]);
        } else if (arg == "--cubeeye-camera-fps") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--cubeeye-camera-fps requires a value");
            }
            options.cubeeye_camera_fps = std::stoi(argv[++i]);
            options.cubeeye_camera_fps_set = true;
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
    if (options.pointcloud_downsample <= 0) {
        throw std::invalid_argument("--pointcloud-downsample must be a positive integer");
    }
    if (options.num_threads <= 0) {
        throw std::invalid_argument("--num-threads must be a positive integer");
    }
    if (!std::isfinite(options.rgb_cubeeye_offset_u) || !std::isfinite(options.rgb_cubeeye_offset_v) ||
        options.rgb_cubeeye_offset_u < -1.0F || options.rgb_cubeeye_offset_u > 1.0F || options.rgb_cubeeye_offset_v < -1.0F ||
        options.rgb_cubeeye_offset_v > 1.0F) {
        throw std::invalid_argument("RGB CubeEye offsets must be between -1.0 and 1.0");
    }
    if (options.cubeeye_camera_fps_set && !is_supported_cubeeye_camera_fps(options.cubeeye_camera_fps)) {
        throw std::invalid_argument("--cubeeye-camera-fps supports S111D values only: 7, 15, 30");
    }
    if (options.viewer_only && options.publisher_type != PublisherType::WebSocket) {
        throw std::invalid_argument("--viewer-only requires --ws");
    }
    if (!uses_rgb_camera(options.camera_input_mode) && options.camera_pipeline_set) {
        throw std::invalid_argument("--camera-pipeline requires RGB camera input");
    }
    if (!options.viewer_only && !uses_rgb_camera(options.camera_input_mode)) {
        throw std::invalid_argument("pick detection requires RGB camera input");
    }
    if (!uses_cubeeye(options.camera_input_mode) && options.cubeeye_frames_set) {
        throw std::invalid_argument("--cubeeye-frames requires CubeEye input");
    }
    if (!uses_cubeeye(options.camera_input_mode) && options.cubeeye_camera_fps_set) {
        throw std::invalid_argument("--cubeeye-camera-fps requires CubeEye input");
    }
    if (options.viewer_only && !options.positional_args.empty()) {
        throw std::invalid_argument("positional arguments are not used with --viewer-only");
    }
    if (options.viewer_only && (!options.hef_path.empty() || !options.metadata_path.empty())) {
        throw std::invalid_argument("model and metadata arguments are not used with --viewer-only");
    }

    return options;
}

AppBootstrap build_app_bootstrap(const AppOptions& options, const char* executable_path) {
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
    bootstrap.processor_config.pointcloud_downsample = options.pointcloud_downsample;
    bootstrap.processor_config.rgb_cubeeye_offset = RgbCubeEyeOffset{
        .u = options.rgb_cubeeye_offset_u,
        .v = options.rgb_cubeeye_offset_v,
    };
    bootstrap.cubeeye_camera_fps = options.cubeeye_camera_fps;
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
    if (uses_cubeeye(options.camera_input_mode)) {
        bootstrap.processor_config.cubeeye_frames = parse_cubeeye_frames(options.cubeeye_frames);
    }
    bootstrap.publisher_type = options.publisher_type;
    bootstrap.websocket_publisher_config.port = options.websocket_port;
    bootstrap.http_api_server_config.port = options.http_port;

    if (uses_rgb_camera(options.camera_input_mode)) {
        const std::string camera_pipeline = options.camera_pipeline.empty() ? std::string(DEFAULT_CAMERA_PIPELINE) : options.camera_pipeline;
        bootstrap.camera_source = catcheye::input::create_frame_source(catcheye::input::InputSourceConfig{
            .type = catcheye::input::InputSourceType::Camera,
            .uri = {},
            .camera_pipeline = camera_pipeline,
            .camera_device = {},
            .camera_width = 1280,
            .camera_height = 720,
        });
    }

    if (!options.viewer_only && options.detector_backend == catcheye::DetectorBackend::Ncnn
        && (ncnn_cfg.param_path.empty() || ncnn_cfg.bin_path.empty())) {
        throw std::runtime_error("NCNN model paths are required");
    }
    if (!options.viewer_only && options.detector_backend == catcheye::DetectorBackend::Hailo && hailo_cfg.hef_path.empty()) {
        throw std::runtime_error("Hailo HEF path is required; pass --hef <model.hef>");
    }

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

    if (options.list_cubeeye_sources) {
        return list_cubeeye_sources();
    }

    AppBootstrap bootstrap = build_app_bootstrap(options, argv[0]);
    std::cerr << "catcheye-pick starting (mode='" << describe_runtime_mode(options) << "', publisher='"
              << publisher_name(bootstrap.publisher_type) << "'";
    if (!options.viewer_only) {
        std::cerr << ", detector='" << detector_backend_name(options.detector_backend) << "'";
    }
    if (uses_cubeeye(options.camera_input_mode)) {
        std::cerr << ", cubeeye_frames='" << options.cubeeye_frames << "'"
                  << ", pointcloud_downsample=" << options.pointcloud_downsample
                  << ", rgb_cubeeye_offset=(" << options.rgb_cubeeye_offset_u << ',' << options.rgb_cubeeye_offset_v << ')';
        if (options.cubeeye_camera_fps_set) {
            std::cerr << ", cubeeye_camera_fps=" << options.cubeeye_camera_fps;
        }
    }
    std::cerr << ")\n";

    if (options.viewer_only) {
        return run_viewer_only(std::move(bootstrap));
    }

    return run_pick_detection(std::move(bootstrap));
}

} // namespace catcheye::pick
