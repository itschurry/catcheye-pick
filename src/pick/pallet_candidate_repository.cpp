#include "pick/pallet_candidate_repository.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "pick/config_json.hpp"

namespace catcheye::pick {

bool is_valid_pallet_candidate_config(const PalletCandidateConfig& config)
{
    return std::isfinite(config.min_height_m) && std::isfinite(config.max_height_m) && config.min_height_m > 0.0F &&
           config.max_height_m > config.min_height_m && config.min_points > 0 && config.max_image_gap_px >= 1;
}

PalletCandidateConfig load_pallet_candidate_config(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to load pallet candidate config: " + path);
    }

    const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    PalletCandidateConfig config;
    if (!parse_json_bool_field(json, "enabled", config.enabled) ||
        !parse_json_float_field(json, "min_height_m", config.min_height_m) ||
        !parse_json_float_field(json, "max_height_m", config.max_height_m) ||
        !parse_json_int_field(json, "min_points", config.min_points) ||
        !parse_json_int_field(json, "max_image_gap_px", config.max_image_gap_px) ||
        !is_valid_pallet_candidate_config(config)) {
        throw std::runtime_error("invalid pallet candidate config: " + path);
    }
    return config;
}

std::string pallet_candidate_config_to_json(const PalletCandidateConfig& config)
{
    std::ostringstream oss;
    oss << "{\n"
        << "  \"enabled\": " << (config.enabled ? "true" : "false") << ",\n"
        << "  \"min_height_m\": " << config.min_height_m << ",\n"
        << "  \"max_height_m\": " << config.max_height_m << ",\n"
        << "  \"min_points\": " << config.min_points << ",\n"
        << "  \"max_image_gap_px\": " << config.max_image_gap_px << "\n"
        << "}\n";
    return oss.str();
}

bool save_pallet_candidate_config(const PalletCandidateConfig& config, const std::string& path)
{
    if (!is_valid_pallet_candidate_config(config)) {
        return false;
    }
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        return false;
    }
    output << pallet_candidate_config_to_json(config);
    return static_cast<bool>(output);
}

} // namespace catcheye::pick
