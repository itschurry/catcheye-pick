#include "pick/viewer_payload_builder.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "catcheye/input/pixel_format.hpp"

namespace catcheye::pick {
namespace {

cv::Mat frame_to_bgr(const catcheye::input::Frame& frame)
{
    if (frame.empty() || frame.width <= 0 || frame.height <= 0 || frame.stride <= 0) {
        return {};
    }

    const std::size_t expected_size = catcheye::input::frame_data_size(frame.format, frame.stride, frame.height);
    if (frame.data.size() < expected_size) {
        return {};
    }

    auto* raw = const_cast<std::uint8_t*>(frame.data.data());
    switch (frame.format) {
    case catcheye::input::PixelFormat::BGR: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        return wrapped.clone();
    }
    case catcheye::input::PixelFormat::RGB: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC3, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::RGBA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::BGRA: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC4, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::GRAY8: {
        cv::Mat wrapped(frame.height, frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    case catcheye::input::PixelFormat::NV12: {
        cv::Mat wrapped(frame.height + (frame.height / 2), frame.width, CV_8UC1, raw, static_cast<std::size_t>(frame.stride));
        cv::Mat bgr;
        cv::cvtColor(wrapped, bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
    }
    case catcheye::input::PixelFormat::UNKNOWN:
        break;
    }
    return {};
}

std::vector<std::uint8_t> encode_jpeg(const cv::Mat& bgr)
{
    if (bgr.empty()) {
        throw std::runtime_error("cannot encode empty frame");
    }
    std::vector<std::uint8_t> jpeg;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", bgr, jpeg, params)) {
        throw std::runtime_error("failed to encode JPEG frame");
    }
    return jpeg;
}

} // namespace

ViewerPayload camera_payload(const catcheye::input::Frame& frame)
{
    const cv::Mat bgr = frame_to_bgr(frame);
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert Camera Module 3 frame");
    }
    return ViewerPayload{
        .name = "realsense_d455_sim",
        .kind = "camera",
        .encoding = "jpeg",
        .width = bgr.cols,
        .height = bgr.rows,
        .point_count = 0,
        .stride = 1,
        .source_timestamp_ms = static_cast<std::uint64_t>(frame.timestamp),
        .bytes = encode_jpeg(bgr),
    };
}

ViewerPayload depth_payload(const catcheye::input::Frame& frame)
{
    const cv::Mat bgr = frame_to_bgr(frame);
    if (bgr.empty()) {
        throw std::runtime_error("failed to convert RealSense D455 depth frame");
    }
    return ViewerPayload{
        .name = "realsense_d455_depth",
        .kind = "depth",
        .encoding = "jpeg",
        .width = bgr.cols,
        .height = bgr.rows,
        .point_count = 0,
        .stride = 1,
        .source_timestamp_ms = static_cast<std::uint64_t>(frame.timestamp),
        .bytes = encode_jpeg(bgr),
    };
}

} // namespace catcheye::pick
