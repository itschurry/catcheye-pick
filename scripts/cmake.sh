#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CATCHEYE_PICK_CONTAINER:-catcheye-pick-develop-raspbian}"
WORKDIR="${CATCHEYE_PICK_CONTAINER_WORKDIR:-/home/user/catcheye-pick}"

usage() {
  cat <<'EOF'
Usage:
  scripts/cmake.sh <command> [profile]

Commands:
  configure   Configure CMake
  build       Build
  install     Install
  verify      Run installed app --help
  clean       Remove build/install and clean log
  all         Configure, build, install, verify

Profiles:
  debug
  release
  release-hailo (default)

Examples:
  scripts/cmake.sh build
  scripts/cmake.sh all release-hailo
  scripts/cmake.sh clean
EOF
}

command="${1:-}"
profile="${2:-release-hailo}"

if [[ -z "$command" || "$command" == "-h" || "$command" == "--help" ]]; then
  usage
  exit 0
fi

build_dir() {
  case "$profile" in
    debug) echo "build/debug" ;;
    release) echo "build/release" ;;
    release-hailo) echo "build/release-hailo" ;;
    *) echo "unknown profile: $profile" >&2; exit 2 ;;
  esac
}

install_dir() {
  case "$profile" in
    debug) echo "install/debug" ;;
    release) echo "install/release" ;;
    release-hailo) echo "install/release-hailo" ;;
    *) echo "unknown profile: $profile" >&2; exit 2 ;;
  esac
}

config_name() {
  case "$profile" in
    debug) echo "Debug" ;;
    release|release-hailo) echo "Release" ;;
    *) echo "unknown profile: $profile" >&2; exit 2 ;;
  esac
}

configure_args() {
  case "$profile" in
    debug) echo "-DCMAKE_BUILD_TYPE=Debug" ;;
    release) echo "-DCMAKE_BUILD_TYPE=Release" ;;
    release-hailo) echo "-DCMAKE_BUILD_TYPE=Release -DCATCHEYE_VISION_DETECTION_ENABLE_HAILO=ON" ;;
    *) echo "unknown profile: $profile" >&2; exit 2 ;;
  esac
}

in_container() {
  docker exec "$CONTAINER" bash -lc "cd '$WORKDIR' && $*"
}

configure() {
  in_container "cmake -S . -B '$(build_dir)' -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(configure_args)"
}

build() {
  in_container "cmake --build '$(build_dir)' --config '$(config_name)' -- -j \$(nproc) && mkdir -p build && ln -sf \$(realpath '$(build_dir)'/compile_commands.json) build/compile_commands.json"
}

install_app() {
  in_container "rm -rf '$(install_dir)' && cmake --install '$(build_dir)' --prefix '$(install_dir)'"
}

verify() {
  in_container "'$(install_dir)'/bin/catcheye-pick --help"
}

clean() {
  in_container "rm -rf build install && find log -mindepth 1 ! -name .gitkeep -delete"
}

case "$command" in
  configure) configure ;;
  build) build ;;
  install) install_app ;;
  verify) verify ;;
  clean) clean ;;
  all)
    configure
    build
    install_app
    verify
    ;;
  *)
    echo "unknown command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
