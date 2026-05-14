#pragma once

#include <string>

#include "pick/processor_config.hpp"

namespace catcheye::pick {

bool is_valid_pallet_candidate_config(const PalletCandidateConfig& config);
PalletCandidateConfig load_pallet_candidate_config(const std::string& path);
std::string pallet_candidate_config_to_json(const PalletCandidateConfig& config);
bool save_pallet_candidate_config(const PalletCandidateConfig& config, const std::string& path);

} // namespace catcheye::pick
