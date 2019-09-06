cmake_minimum_required(VERSION 3.12)

project(ORL
        VERSION 0.1)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

cmake_policy(SET CMP0074 NEW)

find_package(GLFW)
find_package(Vulkan)

include_directories(${GLFW_INCLUDE_DIRS}
                    ${Vulkan_INCLUDE_DIRS})

add_subdirectory(orlviewer)