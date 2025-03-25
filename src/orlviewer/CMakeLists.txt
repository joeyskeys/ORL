include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(viewer_headers
    glfw_application.h
    mesh.h)

set(viewer_srcs
    glfw_application.cpp
    main.cpp)

set(loader_headers
    mesh.h)

set(loader_srcs)

FLEX_BISON(${CMAKE_CURRENT_SOURCE_DIR}/loaders/objlex.l ${CMAKE_CURRENT_SOURCE_DIR}/loaders/objgram.y obj loader_srcs loader_headers)

add_library(loader
    ${loader_headers}
    ${loader_srcs})

add_executable(orlviewer
    ${viewer_headers}
    ${viewer_srcs})

target_link_libraries(orlviewer
    general
        loader
        glfw
        Vulkan::Vulkan)

file(GLOB shader_srcs shaders/*)
set(output_shader_paths "")
add_custom_target(compile_glsl_shaders VERBATIM)
set(shader_output_folder "${CMAKE_CURRENT_BINARY_DIR}/shaders")
if(NOT EXISTS ${shader_output_folder})
    file(MAKE_DIRECTORY ${shader_output_folder})
endif()

foreach(shader_file ${shader_srcs})
    get_filename_component(output_shader_name ${shader_file} NAME_WE)
    set(output_shader_path "${shader_output_folder}/${output_shader_name}.spv")
    add_custom_command(
        TARGET compile_glsl_shaders POST_BUILD
        COMMAND ${Vulkan_GLSLC_EXECUTABLE} -o ${output_shader_path} ${shader_file} && echo "${output_shader_path}"
        MAIN_DEPENDECY ${shader_file}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    list(APPEND output_shader_paths ${output_shader_path})
endforeach()

add_custom_target(compile_shader_target
    DEPENDS ${output_shader_paths}
    VERBATIM)

add_dependencies(compile_shader_target compile_glsl_shaders)
add_dependencies(orlviewer compile_shader_target)