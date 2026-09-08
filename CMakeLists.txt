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
option(ORL_BUILD_VIEWER "Build the Vulkan ORL viewport" ON)
option(advanced_gui "Use Qt6 instead of GLFW for the viewer GUI when Qt6 is available" OFF)

cmake_policy(SET CMP0074 NEW)

include_directories(src/include)

#include(flexbison)
add_subdirectory(src/orlcomp)
add_subdirectory(src/orlexec)

find_package(GLM CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
#find_package(cglm CONFIG QUIET)

# Viewer window backend is an orl_viewer compile choice. vkkk can build GLFW
# and Qt backends together; advanced_gui picks which one the viewer uses.
set(ORL_USE_QT6 OFF)
if(ORL_BUILD_VIEWER AND advanced_gui)
    find_package(Qt6 QUIET COMPONENTS Core Gui Widgets)
    if(Qt6_FOUND)
        set(ORL_USE_QT6 ON)
        message(STATUS "advanced_gui: Qt6 ${Qt6_VERSION} found; orl_viewer will use QtBackend")
    else()
        message(WARNING "advanced_gui is ON but Qt6 was not found; falling back to GLFW")
    endif()
endif()

if(ORL_BUILD_VIEWER)
    set(VKKK_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build vkkk Python bindings" FORCE)
    set(VKKK_VCPKG_INSTALLED_DIR "${CMAKE_SOURCE_DIR}/../vcpkg/installed/x64-windows"
        CACHE PATH "Path to vcpkg installed triplet directory" FORCE)
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

    add_subdirectory(src/orlviewer)
endif()

#add_subdirectory(orltest)

if (BUILD_TESTS)
    find_package(Catch2 CONFIG REQUIRED)
endif()

if (Catch2_FOUND AND BUILD_TESTS)
    add_subdirectory(tests)
endif()
