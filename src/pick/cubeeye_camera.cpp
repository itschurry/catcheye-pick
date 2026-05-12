#include "pick/cubeeye_camera.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace catcheye::pick {
namespace {

constexpr int CUBEEYE_FRAME_WAIT_MS = 5000;
constexpr std::string_view CUBEEYE_FRAMERATE_PROPERTY = "framerate";

constexpr std::array<std::string_view, 27> CUBEEYE_PROPERTY_KEYS{
    "amplitude_threshold_max",
    "amplitude_threshold_min",
    "amplitude_time_filter",
    "amplitude_time_spatial_threshold",
    "amplitude_time_temporal_threshold",
    "auto_exposure",
    "depth_average_median_filter",
    "depth_average_median_max_n",
    "depth_offset",
    "depth_range_max",
    "depth_range_min",
    "depth_time_filter",
    "depth_time_spatial_threshold",
    "depth_time_temporal_threshold",
    "flying_pixel_remove_filter",
    "flying_pixel_remove_threshold",
    "framerate",
    "illumination",
    "integraion_time",
    "motion_blur_frequency",
    "motion_blur_threshold",
    "motion_blur_threshold2",
    "noise_filter1",
    "noise_filter2",
    "noise_filter3",
    "scattering_threshold",
    "temperature",
};

std::string json_escape(std::string_view value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        if (ch == '"' || ch == '\\') {
            escaped.push_back('\\');
        }
        escaped.push_back(ch);
    }
    return escaped;
}

std::string property_json_pair(const meere::sensor::sptr_camera& camera, std::string_view key)
{
    const auto [result, property] = camera->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !property) {
        throw std::runtime_error("failed to get CubeEye property: " + std::string(key));
    }

    std::ostringstream oss;
    oss << "\"" << key << "\":";
    switch (property->dataType()) {
    case meere::sensor::DataType::Boolean:
        oss << (property->asBoolean(false) ? "true" : "false");
        break;
    case meere::sensor::DataType::S8:
        oss << static_cast<int>(property->asInt8s(0));
        break;
    case meere::sensor::DataType::U8:
        oss << static_cast<unsigned int>(property->asInt8u(0));
        break;
    case meere::sensor::DataType::S16:
        oss << property->asInt16s(0);
        break;
    case meere::sensor::DataType::U16:
        oss << property->asInt16u(0);
        break;
    case meere::sensor::DataType::S32:
        oss << property->asInt32s(0);
        break;
    case meere::sensor::DataType::U32:
        oss << property->asInt32u(0);
        break;
    case meere::sensor::DataType::S64:
        oss << property->asInt64s(0);
        break;
    case meere::sensor::DataType::U64:
        oss << property->asInt64u(0);
        break;
    case meere::sensor::DataType::F32:
        oss << property->asFlt32(0);
        break;
    case meere::sensor::DataType::F64:
        oss << property->asFlt64(0);
        break;
    case meere::sensor::DataType::String:
        oss << "\"" << json_escape(property->asString("")) << "\"";
        break;
    default:
        throw std::runtime_error("unsupported CubeEye property type: " + std::string(key));
    }
    return oss.str();
}

template <typename T>
bool in_range(std::int64_t value)
{
    return value >= static_cast<std::int64_t>(std::numeric_limits<T>::min())
        && value <= static_cast<std::int64_t>(std::numeric_limits<T>::max());
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
    latest_ = std::move(frame_set);
    ++sequence_;
    cv_.notify_all();
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
    if (camera_->run(cubeeye_frame_mask(specs_)) != meere::sensor::result::success) {
        throw std::runtime_error("failed to run CubeEye camera");
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
}

std::optional<std::string> CubeEyeCameraSession::properties_json() const
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return std::nullopt;
    }

    std::ostringstream oss;
    oss << "{";
    for (std::size_t i = 0; i < CUBEEYE_PROPERTY_KEYS.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        oss << property_json_pair(camera_, CUBEEYE_PROPERTY_KEYS[i]);
    }
    oss << "}";
    return oss.str();
}

bool CubeEyeCameraSession::set_property(std::string_view key, bool value)
{
    if (!is_supported_cubeeye_property_key(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto [result, current] = camera_->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !current || current->dataType() != meere::sensor::DataType::Boolean) {
        return false;
    }
    const auto property = meere::sensor::make_property_bool(std::string(key), value);
    return property && camera_->setProperty(property) == meere::sensor::result::success;
}

bool CubeEyeCameraSession::set_property(std::string_view key, std::int64_t value)
{
    if (!is_supported_cubeeye_property_key(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto [result, current] = camera_->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !current) {
        return false;
    }
    meere::sensor::sptr_property property;
    switch (current->dataType()) {
    case meere::sensor::DataType::S8:
        if (in_range<meere::sensor::int8s>(value)) {
            property = meere::sensor::make_property_8s(std::string(key), static_cast<meere::sensor::int8s>(value));
        }
        break;
    case meere::sensor::DataType::U8:
        if (in_range<meere::sensor::int8u>(value)) {
            property = meere::sensor::make_property_8u(std::string(key), static_cast<meere::sensor::int8u>(value));
        }
        break;
    case meere::sensor::DataType::S16:
        if (in_range<meere::sensor::int16s>(value)) {
            property = meere::sensor::make_property_16s(std::string(key), static_cast<meere::sensor::int16s>(value));
        }
        break;
    case meere::sensor::DataType::U16:
        if (in_range<meere::sensor::int16u>(value)) {
            property = meere::sensor::make_property_16u(std::string(key), static_cast<meere::sensor::int16u>(value));
        }
        break;
    case meere::sensor::DataType::S32:
        if (in_range<meere::sensor::int32s>(value)) {
            property = meere::sensor::make_property_32s(std::string(key), static_cast<meere::sensor::int32s>(value));
        }
        break;
    case meere::sensor::DataType::U32:
        if (in_range<meere::sensor::int32u>(value)) {
            property = meere::sensor::make_property_32u(std::string(key), static_cast<meere::sensor::int32u>(value));
        }
        break;
    case meere::sensor::DataType::S64:
        property = meere::sensor::make_property_64s(std::string(key), static_cast<meere::sensor::int64s>(value));
        break;
    case meere::sensor::DataType::U64:
        if (value >= 0) {
            property = meere::sensor::make_property_64u(std::string(key), static_cast<meere::sensor::int64u>(value));
        }
        break;
    default:
        return false;
    }
    return property && camera_->setProperty(property) == meere::sensor::result::success;
}

bool CubeEyeCameraSession::set_property(std::string_view key, double value)
{
    if (!is_supported_cubeeye_property_key(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto [result, current] = camera_->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !current) {
        return false;
    }
    meere::sensor::sptr_property property;
    if (current->dataType() == meere::sensor::DataType::F32) {
        property = meere::sensor::make_property_32f(std::string(key), static_cast<meere::sensor::flt32>(value));
    } else if (current->dataType() == meere::sensor::DataType::F64) {
        property = meere::sensor::make_property_64f(std::string(key), static_cast<meere::sensor::flt64>(value));
    } else {
        return false;
    }
    return property && camera_->setProperty(property) == meere::sensor::result::success;
}

bool CubeEyeCameraSession::set_property(std::string_view key, std::string_view value)
{
    if (!is_supported_cubeeye_property_key(key)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_) {
        return false;
    }
    const auto [result, current] = camera_->getProperty(std::string(key));
    if (result != meere::sensor::result::success || !current || current->dataType() != meere::sensor::DataType::String) {
        return false;
    }
    const auto property = meere::sensor::make_property_string(std::string(key), std::string(value));
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

bool is_supported_cubeeye_property_key(std::string_view key)
{
    return std::find(CUBEEYE_PROPERTY_KEYS.begin(), CUBEEYE_PROPERTY_KEYS.end(), key) != CUBEEYE_PROPERTY_KEYS.end();
}

} // namespace catcheye::pick
