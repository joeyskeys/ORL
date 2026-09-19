if (NOT TARGET vkkk)
    return()
endif()

add_executable(orl_vkkk_tests
    shader_cache_tests.cpp
)

target_include_directories(orl_vkkk_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src
)

target_link_libraries(orl_vkkk_tests
    PRIVATE
        Catch2::Catch2WithMain
        vkkk
)
