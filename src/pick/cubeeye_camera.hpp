#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "CubeEyeCamera.h"
#include "CubeEyeFrameList.h"
#include "CubeEyeSink.h"
#include "CubeEyeSource.h"
#include "pick/processor_config.hpp"

namespace catcheye::pick {

struct CubeEyeFrameEntry {
    CubeEyeFrameSpec spec;
    meere::sensor::sptr_frame frame;
};

struct CubeEyeIntrinsics {
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
};

struct CubeEyeFrameSet {
    std::uint64_t sequence = 0;
    std::vector<CubeEyeFrameEntry> frames;
    std::optional<CubeEyeIntrinsics> intrinsics;
};

class CubeEyeCameraSession final {
  public:
    explicit CubeEyeCameraSession(std::vector<CubeEyeFrameSpec> specs, int camera_fps = 0);
    ~CubeEyeCameraSession();

    CubeEyeCameraSession(const CubeEyeCameraSession&) = delete;
    CubeEyeCameraSession& operator=(const CubeEyeCameraSession&) = delete;

    void open();
    CubeEyeFrameSet read();
    void close();
    std::optional<std::string> properties_json() const;
    bool set_bool_property(std::string_view key, bool value);
    bool set_int_property(std::string_view key, int value);

  private:
    class CaptureSink final : public meere::sensor::sink {
      public:
        explicit CaptureSink(std::vector<CubeEyeFrameSpec> specs);

        std::string name() const override;
        void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::CameraState state) override;
        void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::CameraError error) override;
        void onCubeEyeFrameList(const meere::sensor::ptr_source source, const meere::sensor::sptr_frame_list& frames) override;

        void set_intrinsics(CubeEyeIntrinsics intrinsics);
        CubeEyeFrameSet wait_for_frames(std::uint64_t last_sequence);
        std::uint64_t sequence() const;

      private:
        std::vector<CubeEyeFrameSpec> specs_;
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::optional<CubeEyeFrameSet> latest_;
        std::optional<CubeEyeIntrinsics> intrinsics_;
        std::optional<std::string> error_;
        std::uint64_t sequence_ = 0;
    };

    std::vector<CubeEyeFrameSpec> specs_;
    int camera_fps_ = 0;
    CaptureSink capture_;
    meere::sensor::sptr_camera camera_;
    mutable std::mutex camera_mutex_;
};

int list_cubeeye_sources();

} // namespace catcheye::pick
