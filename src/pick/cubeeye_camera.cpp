#include "pick/cubeeye_camera.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace catcheye::pick {
namespace {

constexpr int CUBEEYE_FRAME_WAIT_MS = 5000;
constexpr std::string_view CUBEEYE_FRAMERATE_PROPERTY = "framerate";

enum class CubeEyePropertyType {
    Bool,
    Int,
    Float,
};

struct CubeEyePropertySpec {
    std::string_view key;
    CubeEyePropertyType type;
    bool required = false;
};

constexpr std::array<CubeEyePropertySpec, 26> CUBEEYE_PROPERTIES{{
    {"framerate", CubeEyePropertyType::Int, true},
    {"auto_exposure", CubeEyePropertyType::Bool, true},
    {"illumination", CubeEyePropertyType::Bool, true},
    {"depth_range_min", CubeEyePropertyType::Int, true},
    {"depth_range_max", CubeEyePropertyType::Int, true},
    {"amplitude_time_filter", CubeEyePropertyType::Bool},
    {"depth_average_median_filter", CubeEyePropertyType::Bool},
    {"depth_time_filter", CubeEyePropertyType::Bool},
    {"flying_pixel_remove_filter", CubeEyePropertyType::Bool},
    {"noise_filter1", CubeEyePropertyType::Bool},
    {"noise_filter2", CubeEyePropertyType::Bool},
    {"noise_filter3", CubeEyePropertyType::Bool},
    {"amplitude_threshold_min", CubeEyePropertyType::Int},
    {"amplitude_threshold_max", CubeEyePropertyType::Int},
    {"amplitude_time_spatial_threshold", CubeEyePropertyType::Float},
    {"amplitude_time_temporal_threshold", CubeEyePropertyType::Float},
    {"depth_average_median_max_n", CubeEyePropertyType::Int},
    {"depth_offset", CubeEyePropertyType::Int},
    {"depth_time_spatial_threshold", CubeEyePropertyType::Float},
    {"depth_time_temporal_threshold", CubeEyePropertyType::Float},
    {"flying_pixel_remove_threshold", CubeEyePropertyType::Int},
    {"integration_time", CubeEyePropertyType::Int},
    {"motion_blur_frequency", CubeEyePropertyType::Int},
    {"motion_blur_threshold", CubeEyePropertyType::Int},
    {"motion_blur_threshold2", CubeEyePropertyType::Int},
    {"scattering_threshold", CubeEyePropertyType::Int},
}};

std::optional<CubeEyePropertySpec> find_property_spec(std::string_view key)
{
    for (const auto& spec : CUBEEYE_PROPERTIES) {
        if (spec.key == key) {
            return spec;
        }
    }
    return std::nullopt;
}

bool is_bool_property(std::string_view key)
{
    const auto spec = find_property_spec(key);
    return spec.has_value() && spec->type == CubeEyePropertyType::Bool;
}

bool is_int_property(std::string_view key)
{
    const auto spec = find_property_spec(key);
    return spec.has_value() && spec->type == CubeEyePropertyType::Int;
}

bool is_float_property(std::string_view key)
{
    const auto spec = find_property_spec(key);
    return spec.has_value() && spec->type == CubeEyePropertyType::Float;
}

std::optional<std::string> property_json_pair(const meere::sensor::sptr_camera& camera, CubeEyePropertySpec spec)
{
    const auto [result, property] = camera->getProperty(std::string(spec.key));
    if (result != meere::sensor::result::success || !property) {
        if (spec.required) {
            throw std::runtime_error("failed to get CubeEye property: " + std::string(spec.key));
        }
        return std::nullopt;
    }
    std::ostringstream oss;
    oss << "\"" << spec.key << "\":";
    if (spec.type == CubeEyePropertyType::Bool) {
        oss << (property->asBoolean(false) ? "true" : "false");
    } else if (spec.type == CubeEyePropertyType::Float) {
        oss << property->asFlt32(0.0F);
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
    bool first = true;
    for (const auto& spec : CUBEEYE_PROPERTIES) {
        const auto pair = property_json_pair(camera_, spec);
        if (!pair.has_value()) {
            continue;
        }
        if (!first) {
            oss << ',';
        }
        first = false;
        oss << *pair;
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

bool CubeEyeCameraSession::set_float_property(std::string_view key, float value)
{
    if (!is_float_property(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto property = meere::sensor::make_property_32f(std::string(key), value);
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
