#include "pick/app.hpp"

#include <iostream>
#include <stdexcept>
#include <string_view>

#include "catcheye/detection/detector_factory.hpp"
#include "CubeEyeSource.h"
#include "CubeEyeCamera.h"

namespace catcheye::pick {
namespace {

void print_usage()
{
    std::cout
        << "Usage: catcheye-pick [options]\n"
        << "\n"
        << "Options:\n"
        << "  --help              Show this help\n"
        << "  --version           Show CubeEye SDK version\n"
        << "  --list-cubeye       List connected CubeEye camera sources\n"
        << "  --detector <name>   Detector backend: ncnn | hailo\n";
}

DetectorBackend parse_detector_backend(std::string_view value)
{
    if (value == "ncnn") {
        return DetectorBackend::Ncnn;
    }
    if (value == "hailo") {
        return DetectorBackend::Hailo;
    }

    throw std::invalid_argument("unknown detector backend: " + std::string(value));
}

int list_cubeye_sources()
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

} // namespace

AppOptions parse_app_options(int argc, char** argv)
{
    AppOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);

        if (arg == "--help" || arg == "-h") {
            options.show_help = true;
        } else if (arg == "--version") {
            options.show_version = true;
        } else if (arg == "--list-cubeye") {
            options.list_cubeye_sources = true;
        } else if (arg == "--detector") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--detector requires a value");
            }
            options.detector_backend = parse_detector_backend(argv[++i]);
        } else {
            options.positional_args.emplace_back(arg);
        }
    }

    return options;
}

int run_app(int argc, char** argv)
{
    const auto options = parse_app_options(argc, argv);

    if (options.show_help) {
        print_usage();
        return 0;
    }

    if (options.show_version) {
        std::cout << "CubeEye SDK " << meere::sensor::last_released_version()
                  << " (" << meere::sensor::last_released_date() << ")\n";
        return 0;
    }

    if (options.list_cubeye_sources) {
        return list_cubeye_sources();
    }

    print_usage();
    return 0;
}

} // namespace catcheye::pick
