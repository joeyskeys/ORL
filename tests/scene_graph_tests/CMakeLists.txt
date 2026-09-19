if (NOT TARGET orlrig OR NOT TARGET vkkk)
    return()
endif()

add_executable(orl_scene_graph_tests
    scene_graph_tests.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/component_manager.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/control_map.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/ops/create_joint_op.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/ops/mirror_op.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/ops/delete_op.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/graph_scene_inputs.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/scene_graph_context.cpp
    ${CMAKE_SOURCE_DIR}/src/orlviewer/graph_scene_runtime.cpp
)

target_include_directories(orl_scene_graph_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src/orlviewer
        ${CMAKE_SOURCE_DIR}/src/include
        ${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src
)

target_link_libraries(orl_scene_graph_tests
    PRIVATE
        Catch2::Catch2WithMain
        vkkk
        orlexec
        orlrig
        Eigen3::Eigen
)
