#pragma once

#include <string>
#include <vector>

namespace catcheye::pick {

enum class DetectorBackend {
    Ncnn,
    Hailo,
};

struct AppOptions {
    bool show_help = false;
    bool show_version = false;
    bool list_cubeye_sources = false;
    DetectorBackend detector_backend = DetectorBackend::Ncnn;
    std::vector<std::string> positional_args;
};

AppOptions parse_app_options(int argc, char** argv);
int run_app(int argc, char** argv);

} // namespace catcheye::pick
