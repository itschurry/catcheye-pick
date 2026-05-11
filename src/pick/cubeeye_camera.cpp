#include "pick/cubeeye_camera.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace catcheye::pick {
namespace {

constexpr int CUBEEYE_FRAME_WAIT_MS = 5000;

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

CubeEyeCameraSession::CubeEyeCameraSession(std::vector<CubeEyeFrameSpec> specs)
    : specs_(std::move(specs)),
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
