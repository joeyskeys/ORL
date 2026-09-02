add_executable(orl_viewer
    main.cpp
    control_map.cpp
    component_manager.cpp
    comps/joint.hpp
    comps/joint.glsl
    comps/weight.hpp
    comps/deformer.hpp
    ops/load_model_op.cpp
    ops/create_joint_op.cpp
    vp/mesh_csr_feature.cpp
    vp/auto_weight_feature.cpp
    vp/deformer_feature.cpp
)
set_source_files_properties(comps/joint.hpp comps/joint.glsl comps/weight.hpp comps/deformer.hpp PROPERTIES HEADER_FILE_ONLY TRUE)

if(VKKK_VCPKG_INSTALLED_DIR)
    list(PREPEND CMAKE_PREFIX_PATH "${VKKK_VCPKG_INSTALLED_DIR}")
endif()
find_package(RapidJSON CONFIG REQUIRED)

target_link_libraries(orl_viewer
    PRIVATE
        vkkk
        orlexec
        Eigen3::Eigen
        RapidJSON
)
if(WIN32)
    target_link_libraries(orl_viewer PRIVATE comdlg32)
    target_compile_definitions(orl_viewer PRIVATE NOMINMAX)
endif()

target_include_directories(orl_viewer
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_SOURCE_DIR}/src/include"
        "${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src"
        "${CMAKE_SOURCE_DIR}/thirdparty/vkkk/thirdparty/imgui"
)

target_compile_definitions(orl_viewer
    PRIVATE
        ORL_VKKK_SOURCE_DIR="${CMAKE_SOURCE_DIR}/thirdparty/vkkk"
        ORL_RESOURCE_DIR="${CMAKE_SOURCE_DIR}/resource"
)
