if (NOT TARGET orlcomp_codegen)
    return()
endif()

add_executable(orl_gpu_tests
    gpu_tests.cpp
)

target_link_libraries(orl_gpu_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_codegen
)
