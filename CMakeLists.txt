cmake_minimum_required(VERSION 3.20)

project(catcheye-pick
    VERSION 0.1.0
    DESCRIPTION "CatchEye Pick"
    LANGUAGES CXX
)

option(CATCHEYE_PICK_BUILD_TESTS "Build tests" OFF)
option(CATCHEYE_PICK_BUILD_APP "Build main executable" ON)
option(CATCHEYE_PICK_BUNDLE_RUNTIME_LIBS "Install runtime shared libraries with the app" ON)

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

set(CUBEYE_SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CubeEye2.0-sdk")
set(CUBEYE_INCLUDE_DIR "${CUBEYE_SDK_SOURCE_DIR}/include")
set(CUBEYE_LIBRARY "${CUBEYE_SDK_SOURCE_DIR}/lib/libCubeEye.so")
set(CUBEYE_RUNTIME_LIBRARY_DIRS
    "${CUBEYE_SDK_SOURCE_DIR}/thirdparty/libffmpeg/lib"
    "${CUBEYE_SDK_SOURCE_DIR}/thirdparty/liblive555/lib/Release"
    "${CUBEYE_SDK_SOURCE_DIR}/thirdparty/libusb/lib"
)
if(NOT EXISTS "${CUBEYE_INCLUDE_DIR}/CubeEye/CubeEyeCamera.h")
    message(FATAL_ERROR "CubeEye SDK headers not found at ${CUBEYE_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${CUBEYE_LIBRARY}")
    message(FATAL_ERROR "CubeEye runtime library not found at ${CUBEYE_LIBRARY}")
endif()

set(CUBEYE_RUNTIME_LIBRARIES "${CUBEYE_LIBRARY}")
set(CUBEYE_LINK_LIBRARIES)
foreach(cubeye_runtime_library_dir IN LISTS CUBEYE_RUNTIME_LIBRARY_DIRS)
    if(EXISTS "${cubeye_runtime_library_dir}")
        file(GLOB cubeye_runtime_libraries
            "${cubeye_runtime_library_dir}/*.so"
            "${cubeye_runtime_library_dir}/*.so.*"
        )
        list(APPEND CUBEYE_RUNTIME_LIBRARIES ${cubeye_runtime_libraries})

        file(GLOB cubeye_link_libraries
            "${cubeye_runtime_library_dir}/*.so"
            "${cubeye_runtime_library_dir}/*.so.[0-9]*"
        )
        list(APPEND CUBEYE_LINK_LIBRARIES ${cubeye_link_libraries})
    endif()
endforeach()
list(REMOVE_DUPLICATES CUBEYE_RUNTIME_LIBRARIES)
list(REMOVE_DUPLICATES CUBEYE_LINK_LIBRARIES)

set(CATCHEYE_VISION_SDK_BUILD_TESTS OFF CACHE BOOL "Build tests for SDK libraries" FORCE)

find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn imgcodecs videoio)
find_package(spdlog REQUIRED)

add_subdirectory(
    "${CATCHEYE_VISION_SDK_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/catcheye-vision-sdk"
)

add_library(cubeye_sdk SHARED IMPORTED GLOBAL)
set_target_properties(cubeye_sdk PROPERTIES
    IMPORTED_LOCATION "${CUBEYE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CUBEYE_INCLUDE_DIR}/CubeEye"
    INTERFACE_LINK_LIBRARIES "${CUBEYE_LINK_LIBRARIES}"
    INTERFACE_LINK_OPTIONS "$<$<CXX_COMPILER_ID:GNU,Clang>:-Wl,-rpath-link,${CUBEYE_SDK_SOURCE_DIR}/thirdparty/libffmpeg/lib;-Wl,-rpath-link,${CUBEYE_SDK_SOURCE_DIR}/thirdparty/liblive555/lib/Release;-Wl,-rpath-link,${CUBEYE_SDK_SOURCE_DIR}/thirdparty/libusb/lib>"
)
add_library(cubeye::sdk ALIAS cubeye_sdk)

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
        "${CUBEYE_INCLUDE_DIR}/CubeEye"
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
        cubeye::sdk
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
            cubeye::sdk
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
        target_link_libraries(catcheye_pick_tests PRIVATE catcheye::pick cubeye::sdk)
        target_include_directories(catcheye_pick_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        add_test(NAME catcheye_pick_tests COMMAND catcheye_pick_tests)
    endif()
endif()

include(GNUInstallDirs)

set(CATCHEYE_PICK_LAUNCHER "${CMAKE_CURRENT_BINARY_DIR}/catcheye-pick")
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

install(DIRECTORY "${CUBEYE_INCLUDE_DIR}/CubeEye"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CATCHEYE_VISION_SDK_SOURCE_DIR}/libs/vision-input/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
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

install(DIRECTORY models
    DESTINATION ${CMAKE_INSTALL_DATADIR}/..
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

if(CATCHEYE_PICK_BUNDLE_RUNTIME_LIBS)
    install(FILES ${CUBEYE_RUNTIME_LIBRARIES}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    set(CATCHEYE_NCNN_LIBRARY_DIRS)
    if(CMAKE_LIBRARY_ARCHITECTURE)
        list(APPEND CATCHEYE_NCNN_LIBRARY_DIRS
            "/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
            "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        )
    endif()
    list(APPEND CATCHEYE_NCNN_LIBRARY_DIRS "/usr/local/lib" "/usr/lib")

    foreach(ncnn_library_dir IN LISTS CATCHEYE_NCNN_LIBRARY_DIRS)
        if(EXISTS "${ncnn_library_dir}")
            file(GLOB ncnn_libraries
                "${ncnn_library_dir}/libncnn*.so"
                "${ncnn_library_dir}/libncnn*.so.*"
            )
            list(APPEND CATCHEYE_NCNN_LIBRARIES ${ncnn_libraries})
        endif()
    endforeach()

    if(CATCHEYE_NCNN_LIBRARIES)
        list(REMOVE_DUPLICATES CATCHEYE_NCNN_LIBRARIES)
        install(FILES ${CATCHEYE_NCNN_LIBRARIES} DESTINATION ${CMAKE_INSTALL_LIBDIR})
    endif()
endif()

install(EXPORT catcheye_pick_targets
    FILE catcheye_pick_targets.cmake
    NAMESPACE catcheye::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/catcheye_pick
)
