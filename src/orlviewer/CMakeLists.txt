add_executable(orl_viewer
    main.cpp
    control_map.cpp
)

if(VKKK_VCPKG_INSTALLED_DIR)
    list(PREPEND CMAKE_PREFIX_PATH "${VKKK_VCPKG_INSTALLED_DIR}")
endif()
find_package(RapidJSON CONFIG REQUIRED)

target_link_libraries(orl_viewer
    PRIVATE
        vkkk
        Eigen3::Eigen
        RapidJSON
)

target_include_directories(orl_viewer
    PRIVATE
        "${CMAKE_SOURCE_DIR}/src/include"
        "${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src"
)

target_compile_definitions(orl_viewer
    PRIVATE
        ORL_VKKK_SOURCE_DIR="${CMAKE_SOURCE_DIR}/thirdparty/vkkk"
        ORL_RESOURCE_DIR="${CMAKE_SOURCE_DIR}/resource"
)
