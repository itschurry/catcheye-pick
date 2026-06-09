cmake_minimum_required(VERSION 3.20)

project(catcheye-pick
    VERSION 0.1.0
    DESCRIPTION "CatchEye Pick"
    LANGUAGES CXX
)

option(CATCHEYE_PICK_BUILD_TESTS "Build tests" OFF)
option(CATCHEYE_PICK_BUILD_APP "Build main executable" ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(CATCHEYE_PICK_HARDWARE_DEFAULT ON)
else()
    set(CATCHEYE_PICK_HARDWARE_DEFAULT OFF)
endif()

option(CATCHEYE_PICK_ENABLE_LIBCAMERA "Build Raspberry Pi/Orin libcamera input backend" ${CATCHEYE_PICK_HARDWARE_DEFAULT})
option(CATCHEYE_VISION_DETECTION_ENABLE_HAILO "Build HailoRT detector backend" ${CATCHEYE_PICK_HARDWARE_DEFAULT})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )
elseif(MSVC)
    add_compile_options(/W4)
endif()

set(CATCHEYE_VISION_SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catcheye-vision-sdk")
if(NOT EXISTS "${CATCHEYE_VISION_SDK_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "catcheye-vision-sdk repo not found at ${CATCHEYE_VISION_SDK_SOURCE_DIR}")
endif()

set(CATCHEYE_VISION_SDK_BUILD_TESTS OFF CACHE BOOL "Build tests for SDK libraries" FORCE)
set(CATCHEYE_VISION_INPUT_ENABLE_LIBCAMERA ${CATCHEYE_PICK_ENABLE_LIBCAMERA} CACHE BOOL "Build libcamera input backend" FORCE)

find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn imgcodecs videoio)
find_package(spdlog REQUIRED)

add_subdirectory(
    "${CATCHEYE_VISION_SDK_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/catcheye-vision-sdk"
)

file(GLOB_RECURSE CATCHEYE_PICK_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(CATCHEYE_PICK_MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM CATCHEYE_PICK_SOURCES "${CATCHEYE_PICK_MAIN_SOURCE}")

add_library(catcheye_pick STATIC ${CATCHEYE_PICK_SOURCES})
add_library(catcheye::pick ALIAS catcheye_pick)

target_include_directories(catcheye_pick
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(catcheye_pick PUBLIC cxx_std_20)

target_compile_definitions(catcheye_pick
    PUBLIC
        $<$<CONFIG:Debug>:CATCHEYE_DEBUG>
)

target_link_libraries(catcheye_pick
    PUBLIC
        catcheye::vision_input
        catcheye::vision_detection
        catcheye::vision_roi
        catcheye::vision_protocol
        catcheye::vision_runtime
        catcheye::vision_transport
        catcheye::vision_visualization
        catcheye::vision_http
        ${OpenCV_LIBS}
        spdlog::spdlog
)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

if(CATCHEYE_PICK_BUILD_APP)
    add_executable(catcheye-pick "${CATCHEYE_PICK_MAIN_SOURCE}")
    set_target_properties(catcheye-pick PROPERTIES OUTPUT_NAME "catcheye-pick-bin")

    target_link_libraries(catcheye-pick
        PRIVATE
            catcheye::pick
    )

    target_include_directories(catcheye-pick
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_compile_features(catcheye-pick PRIVATE cxx_std_20)
endif()

if(CATCHEYE_PICK_BUILD_TESTS)
    include(CTest)
    enable_testing()

    file(GLOB_RECURSE CATCHEYE_PICK_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
    )

    if(CATCHEYE_PICK_TEST_SOURCES)
        add_executable(catcheye_pick_tests ${CATCHEYE_PICK_TEST_SOURCES})
        target_link_libraries(catcheye_pick_tests PRIVATE catcheye::pick)
        target_include_directories(catcheye_pick_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        add_test(NAME catcheye_pick_tests COMMAND catcheye_pick_tests)
    endif()
endif()

include(GNUInstallDirs)

set(CATCHEYE_PICK_LAUNCHER "${CMAKE_CURRENT_BINARY_DIR}/catcheye-pick")
if(CMAKE_LIBRARY_ARCHITECTURE)
    set(CATCHEYE_GST_PLUGIN_PATH "/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}/gstreamer-1.0:/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/gstreamer-1.0")
else()
    set(CATCHEYE_GST_PLUGIN_PATH "/usr/local/lib/gstreamer-1.0:/usr/lib/gstreamer-1.0")
endif()
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/catcheye-pick-launcher.sh.in"
    "${CATCHEYE_PICK_LAUNCHER}"
    @ONLY
)

install(TARGETS catcheye_vision_input
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_detection
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_roi
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_protocol
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_transport
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_runtime
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_visualization
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_vision_http
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS catcheye_pick
    EXPORT catcheye_pick_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(TARGET catcheye-pick)
    install(TARGETS catcheye-pick
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(PROGRAMS "${CATCHEYE_PICK_LAUNCHER}"
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME catcheye-pick
    )
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(CATCHEYE_PICK_ENABLE_LIBCAMERA)
    install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-input/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
else()
    install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-input/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        PATTERN "libcamera_source.hpp" EXCLUDE
    )
endif()
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-detection/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-roi/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-protocol/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-transport/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-runtime/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-visualization/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-http/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY models
    DESTINATION ${CMAKE_INSTALL_DATADIR}/..
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY assets
    DESTINATION ${CMAKE_INSTALL_DATADIR}/..
    PATTERN ".DS_Store" EXCLUDE
)

install(DIRECTORY config
    DESTINATION ${CMAKE_INSTALL_DATADIR}/..
    PATTERN ".DS_Store" EXCLUDE
)

install(PROGRAMS
    scripts/run-all-hailo.sh
    scripts/run-rgb-hailo.sh
    scripts/run-rgb.sh
    scripts/run-d455-sim.sh
    DESTINATION scripts
)

install(EXPORT catcheye_pick_targets
    FILE catcheye_pick_targets.cmake
    NAMESPACE catcheye::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/catcheye_pick
)
