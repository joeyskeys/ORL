cmake_minimum_required(VERSION 3.12)

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

project(ORL VERSION 0.1)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake)

set(BUILD_TESTS ON)
option(ORL_BUILD_VIEWER "Build the ORL viewer" ON)
option(advanced_gui "Use Qt6 instead of GLFW for the viewer GUI when Qt6 is available" OFF)

set(ORL_RHI_BACKEND "AUTO" CACHE STRING
    "Compile-time ORL graphics adapter selection")
set_property(CACHE ORL_RHI_BACKEND PROPERTY STRINGS AUTO VKKK MT)
string(TOUPPER "${ORL_RHI_BACKEND}" _orl_rhi_backend_input)
if(_orl_rhi_backend_input STREQUAL "AUTO")
    if(APPLE)
        set(ORL_RHI_BACKEND_RESOLVED "MT")
    else()
        set(ORL_RHI_BACKEND_RESOLVED "VKKK")
    endif()
elseif(_orl_rhi_backend_input STREQUAL "VKKK"
    OR _orl_rhi_backend_input STREQUAL "MT")
    set(ORL_RHI_BACKEND_RESOLVED "${_orl_rhi_backend_input}")
else()
    message(FATAL_ERROR
        "ORL_RHI_BACKEND must be AUTO, VKKK, or MT; got "
        "\"${ORL_RHI_BACKEND}\"")
endif()

if(ORL_RHI_BACKEND_RESOLVED STREQUAL "MT" AND NOT APPLE)
    message(FATAL_ERROR
        "ORL_RHI_BACKEND=MT is only supported on macOS")
endif()
if(ORL_RHI_BACKEND_RESOLVED STREQUAL "MT"
    AND CMAKE_VERSION VERSION_LESS 3.26)
    message(FATAL_ERROR
        "The mt backend requires CMake 3.26 or newer")
endif()

if(ORL_BUILD_VIEWER)
    set(_orl_build_rhi_adapters_default ON)
else()
    set(_orl_build_rhi_adapters_default OFF)
endif()
option(ORL_BUILD_RHI_ADAPTERS
    "Build the selected native RHI adapter target"
    ${_orl_build_rhi_adapters_default})
unset(_orl_build_rhi_adapters_default)

if(ORL_BUILD_VIEWER AND NOT ORL_BUILD_RHI_ADAPTERS)
    message(FATAL_ERROR
        "ORL_BUILD_VIEWER requires ORL_BUILD_RHI_ADAPTERS so the selected "
        "RHI target can be linked")
endif()

set(ORL_MT_SOURCE_DIR "" CACHE PATH
    "Optional local mt source tree; otherwise find_package(mt CONFIG REQUIRED)")

if(ORL_RHI_BACKEND_RESOLVED STREQUAL "VKKK")
    set(ORL_RHI_BACKEND_IS_VKKK 1)
    set(ORL_RHI_BACKEND_IS_MT 0)
else()
    set(ORL_RHI_BACKEND_IS_VKKK 0)
    set(ORL_RHI_BACKEND_IS_MT 1)
endif()
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated/orlviewer/rhi")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/orlviewer/rhi/rhi_config.hpp.in"
    "${CMAKE_BINARY_DIR}/generated/orlviewer/rhi/rhi_config.hpp"
    @ONLY)
message(STATUS "ORL RHI backend: ${ORL_RHI_BACKEND_RESOLVED} (static)")

cmake_policy(SET CMP0074 NEW)

include_directories(src/include)

#include(flexbison)
find_package(GLM CONFIG REQUIRED)
add_subdirectory(src/orlgraph)
add_subdirectory(src/orlcomp)
add_subdirectory(src/orlexec)

find_package(Eigen3 CONFIG REQUIRED)
#find_package(cglm CONFIG QUIET)

# The legacy viewer currently uses the vkkk host and feature hierarchy. Keep
# that host choice local to the vkkk build while the new RHI target is selected
# independently at configure time.
set(ORL_USE_QT6 OFF)
if(ORL_BUILD_VIEWER
    AND ORL_RHI_BACKEND_RESOLVED STREQUAL "VKKK"
    AND advanced_gui)
    find_package(Qt6 QUIET COMPONENTS Core Gui Widgets)
    if(Qt6_FOUND)
        set(ORL_USE_QT6 ON)
        message(STATUS "advanced_gui: Qt6 ${Qt6_VERSION} found; orl_viewer will use QtBackend")
    else()
        message(WARNING "advanced_gui is ON but Qt6 was not found; falling back to GLFW")
    endif()
endif()

if(ORL_BUILD_RHI_ADAPTERS
    AND ORL_RHI_BACKEND_RESOLVED STREQUAL "VKKK")
    set(VKKK_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build vkkk Python bindings" FORCE)
    if(NOT DEFINED VKKK_VCPKG_INSTALLED_DIR)
        set(VKKK_VCPKG_INSTALLED_DIR
            "${CMAKE_SOURCE_DIR}/../vcpkg/installed/x64-windows"
            CACHE PATH "Path to vcpkg installed triplet directory")
    endif()
    add_subdirectory(thirdparty/vkkk EXCLUDE_FROM_ALL)

    get_directory_property(_vkkk_has_qt6 DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vkkk"
        DEFINITION VKKK_HAS_QT6)
    get_directory_property(_vkkk_has_glfw DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vkkk"
        DEFINITION VKKK_HAS_GLFW)
    if(ORL_USE_QT6 AND NOT _vkkk_has_qt6)
        message(WARNING "advanced_gui is ON but vkkk did not enable Qt6; falling back to GLFW")
        set(ORL_USE_QT6 OFF)
    endif()
    if(NOT ORL_USE_QT6 AND NOT _vkkk_has_glfw)
        message(FATAL_ERROR "orl_viewer requires GLFW, or enable advanced_gui with Qt6 available")
    endif()
    unset(_vkkk_has_qt6)
    unset(_vkkk_has_glfw)

endif()

if(ORL_BUILD_RHI_ADAPTERS
    AND ORL_RHI_BACKEND_RESOLVED STREQUAL "MT")
    if(ORL_MT_SOURCE_DIR)
        set(MT_BUILD_TESTS OFF CACHE BOOL "Build mt unit tests" FORCE)
        set(MT_BUILD_EXAMPLES OFF CACHE BOOL "Build mt examples" FORCE)
        set(MT_BUILD_APPKIT OFF CACHE BOOL "Build mt AppKit adapter" FORCE)
        set(MT_BUILD_MTKVIEW OFF CACHE BOOL "Build mt MTKView adapter" FORCE)
        add_subdirectory("${ORL_MT_SOURCE_DIR}"
            "${CMAKE_BINARY_DIR}/_deps/mt"
            EXCLUDE_FROM_ALL)
    else()
        find_package(mt CONFIG REQUIRED)
    endif()
endif()

add_subdirectory(src/orlviewer/rhi)

if(ORL_BUILD_VIEWER AND ORL_RHI_BACKEND_RESOLVED STREQUAL "VKKK")
    add_subdirectory(src/orlviewer)
elseif(ORL_BUILD_VIEWER AND ORL_RHI_BACKEND_RESOLVED STREQUAL "MT")
    message(WARNING
        "The legacy orl_viewer is still vkkk/Vulkan-coupled and is not "
        "compiled for the MT configuration yet. Build the RHI contract and "
        "adapter targets while the viewer feature migration proceeds.")
endif()

#add_subdirectory(orltest)

if (BUILD_TESTS)
    find_package(Catch2 CONFIG REQUIRED)
endif()

if (Catch2_FOUND AND BUILD_TESTS)
    add_subdirectory(tests)
endif()
