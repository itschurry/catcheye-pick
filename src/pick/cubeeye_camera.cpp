#include "pick/cubeeye_camera.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace catcheye::pick {
namespace {

constexpr int CUBEEYE_FRAME_WAIT_MS = 5000;
constexpr std::string_view CUBEEYE_FRAMERATE_PROPERTY = "framerate";

bool is_bool_property(std::string_view key)
{
    return key == "auto_exposure" || key == "illumination";
}

bool is_int_property(std::string_view key)
{
    return key == "framerate" || key == "depth_range_min" || key == "depth_range_max";
}

std::string property_json_pair(const meere::sensor::sptr_camera& camera, std::string_view key)
{
    const auto [result, property] = camera->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !property) {
        throw std::runtime_error("failed to get CubeEye property: " + std::string(key));
    }
    std::ostringstream oss;
    oss << "\"" << key << "\":";
    if (is_bool_property(key)) {
        oss << (property->asBoolean(false) ? "true" : "false");
    } else {
        oss << property->asInt32u(0);
    }
    return oss.str();
}

} // namespace

CubeEyeCameraSession::CaptureSink::CaptureSink(std::vector<CubeEyeFrameSpec> specs)
    : specs_(std::move(specs)) {}

std::string CubeEyeCameraSession::CaptureSink::name() const
{
    return "catcheye-pick-cubeeye";
}

void CubeEyeCameraSession::CaptureSink::onCubeEyeCameraState(
    const meere::sensor::ptr_source source,
    meere::sensor::CameraState state)
{
    std::cerr << "CubeEye source " << (source ? source->uri() : "") << " state=" << static_cast<int>(state) << '\n';
}

void CubeEyeCameraSession::CaptureSink::onCubeEyeCameraError(
    const meere::sensor::ptr_source source,
    meere::sensor::CameraError error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    error_ = "CubeEye source " + std::string(source ? source->uri() : "") + " error=" + std::to_string(static_cast<int>(error));
    cv_.notify_all();
}

void CubeEyeCameraSession::CaptureSink::onCubeEyeFrameList(
    const meere::sensor::ptr_source,
    const meere::sensor::sptr_frame_list& frames)
{
    CubeEyeFrameSet frame_set;
    frame_set.frames.reserve(specs_.size());

    for (const auto& spec : specs_) {
        const auto frame = meere::sensor::find_frame(frames, spec.type);
        if (!frame) {
            return;
        }

        const auto copied_frame = meere::sensor::copy_frame(frame);
        if (!copied_frame) {
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = "failed to copy CubeEye " + cubeeye_frame_label(spec.type) + " frame";
            cv_.notify_all();
            return;
        }

        frame_set.frames.push_back({
            .spec = spec,
            .frame = copied_frame,
        });
    }

    std::lock_guard<std::mutex> lock(mutex_);
    frame_set.sequence = ++sequence_;
    frame_set.intrinsics = intrinsics_;
    latest_ = std::move(frame_set);
    cv_.notify_all();
}

void CubeEyeCameraSession::CaptureSink::set_intrinsics(CubeEyeIntrinsics intrinsics)
{
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_ = intrinsics;
}

CubeEyeFrameSet CubeEyeCameraSession::CaptureSink::wait_for_frames(std::uint64_t last_sequence)
{
    std::unique_lock<std::mutex> lock(mutex_);
    const auto ready = cv_.wait_for(lock, std::chrono::milliseconds(CUBEEYE_FRAME_WAIT_MS), [&] {
        return sequence_ != last_sequence || error_.has_value();
    });
    if (!ready) {
        throw std::runtime_error("timed out waiting for selected CubeEye frames");
    }
    if (error_) {
        throw std::runtime_error(*error_);
    }
    return *latest_;
}

std::uint64_t CubeEyeCameraSession::CaptureSink::sequence() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sequence_;
}

CubeEyeCameraSession::CubeEyeCameraSession(std::vector<CubeEyeFrameSpec> specs, int camera_fps)
    : specs_(std::move(specs)),
      camera_fps_(camera_fps),
      capture_(specs_) {}

CubeEyeCameraSession::~CubeEyeCameraSession()
{
    close();
}

void CubeEyeCameraSession::open()
{
    const auto sources = meere::sensor::search_camera_source();
    if (!sources || sources->size() != 1) {
        throw std::runtime_error("catcheye-pick requires exactly one CubeEye camera");
    }

    camera_ = meere::sensor::create_camera(sources->front());
    if (!camera_) {
        throw std::runtime_error("failed to create CubeEye camera");
    }

    if (camera_->addSink(&capture_) != meere::sensor::result::success) {
        throw std::runtime_error("failed to add CubeEye sink");
    }
    if (camera_->prepare() != meere::sensor::result::success) {
        throw std::runtime_error("failed to prepare CubeEye camera");
    }
    meere::sensor::IntrinsicParameters intrinsics;
    if (camera_->intrinsicParameters(intrinsics) == meere::sensor::result::success) {
        capture_.set_intrinsics(CubeEyeIntrinsics{
            .fx = intrinsics.focal.fx,
            .fy = intrinsics.focal.fy,
            .cx = intrinsics.principal.cx,
            .cy = intrinsics.principal.cy,
        });
        std::cerr << "CubeEye intrinsics fx=" << intrinsics.focal.fx
                  << " fy=" << intrinsics.focal.fy
                  << " cx=" << intrinsics.principal.cx
                  << " cy=" << intrinsics.principal.cy << '\n';
    }
    if (camera_fps_ > 0) {
        const auto property = meere::sensor::make_property_8u(
            std::string(CUBEEYE_FRAMERATE_PROPERTY),
            static_cast<meere::sensor::int8u>(camera_fps_));
        if (!property || camera_->setProperty(property) != meere::sensor::result::success) {
            throw std::runtime_error("failed to set CubeEye framerate");
        }
        std::cerr << "CubeEye framerate set to " << camera_fps_ << " fps\n";
    }
    if (camera_->run(cubeeye_frame_mask(specs_)) != meere::sensor::result::success) {
        throw std::runtime_error("failed to run CubeEye camera");
    }
}

std::optional<std::string> CubeEyeCameraSession::properties_json() const
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return std::nullopt;
    }

    std::ostringstream oss;
    oss << "{";
    const std::array<std::string_view, 5> keys{
        "framerate",
        "auto_exposure",
        "illumination",
        "depth_range_min",
        "depth_range_max",
    };
    for (std::size_t i = 0; i < keys.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        oss << property_json_pair(camera_, keys[i]);
    }
    oss << "}";
    return oss.str();
}

bool CubeEyeCameraSession::set_bool_property(std::string_view key, bool value)
{
    if (!is_bool_property(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto property = meere::sensor::make_property_bool(std::string(key), value);
    return property && camera_->setProperty(property) == meere::sensor::result::success;
}

bool CubeEyeCameraSession::set_int_property(std::string_view key, int value)
{
    if (!is_int_property(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    meere::sensor::sptr_property property;
    if (key == "framerate") {
        property = meere::sensor::make_property_8u(std::string(key), static_cast<meere::sensor::int8u>(value));
    } else {
        property = meere::sensor::make_property_16u(std::string(key), static_cast<meere::sensor::int16u>(value));
    }
    return property && camera_->setProperty(property) == meere::sensor::result::success;
}

CubeEyeFrameSet CubeEyeCameraSession::read()
{
    const std::uint64_t last_sequence = capture_.sequence();
    return capture_.wait_for_frames(last_sequence);
}

void CubeEyeCameraSession::close()
{
    if (!camera_) {
        return;
    }
    camera_->stop();
    camera_->removeSink(&capture_);
    meere::sensor::destroy_camera(camera_);
    camera_.reset();
}

int list_cubeeye_sources()
{
    const auto sources = meere::sensor::search_camera_source();
    if (!sources || sources->size() == 0) {
        std::cout << "CubeEye camera not found\n";
        return 1;
    }

    int index = 0;
    for (const auto& source : *sources) {
        std::cout << index++ << ": "
                  << source->name()
                  << " serial=" << source->serialNumber()
                  << " uri=" << source->uri()
                  << '\n';
    }

    return 0;
}

} // namespace catcheye::pick
