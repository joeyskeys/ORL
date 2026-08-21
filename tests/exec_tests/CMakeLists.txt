if (NOT TARGET orlexec)
    return()
endif()

add_executable(orl_exec_tests
    exec_tests.cpp
)

target_link_libraries(orl_exec_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlexec
)
