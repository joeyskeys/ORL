cmake_minimum_required(VERSION 3.12)

project(ORL
        VERSION 0.1)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake)

set(BUILD_TESTS ON)

cmake_policy(SET CMP0074 NEW)

include_directories(src/include)

include(flexbison)

find_package(glfw3)
find_package(GLM)
find_package(Eigen3 REQUIRED)
find_package(Vulkan)

if(Vulkan_FOUND)
    include_directories(${GLFW_INCLUDE_DIRS}
                        ${Vulkan_INCLUDE_DIRS}
                        ${GLM_INCLUDE_DIRS})
    add_subdirectory(src/orlviewer)
endif()

#add_subdirectory(orltest)

find_package(Catch2 3)
if (Catch2_FOUND AND BUILD_TESTS)
    add_subdirectory(tests)
endif()