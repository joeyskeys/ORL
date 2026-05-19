if (NOT TARGET orlcomp_codegen)
    return()
endif()

add_executable(orl_codegen_tests
    codegen_tests.cpp
)

target_link_libraries(orl_codegen_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_codegen
)
