if (NOT TARGET orlcomp_codegen)
    return()
endif()

add_executable(orl_skinning_tests
    skinning_tests.cpp
)

target_link_libraries(orl_skinning_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_codegen
)
