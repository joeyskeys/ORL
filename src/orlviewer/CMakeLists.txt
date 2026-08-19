add_executable(orl_viewer
    main.cpp
)

target_link_libraries(orl_viewer
    PRIVATE
        vkkk
        Eigen3::Eigen
)

target_include_directories(orl_viewer
    PRIVATE
        "${CMAKE_SOURCE_DIR}/src/include"
        "${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src"
)

target_compile_definitions(orl_viewer
    PRIVATE
        ORL_VKKK_SOURCE_DIR="${CMAKE_SOURCE_DIR}/thirdparty/vkkk"
)
