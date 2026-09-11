if (NOT TARGET orlrig)
    return()
endif()

add_executable(orlrig_tests
    orlrig_tests.cpp
)

target_link_libraries(orlrig_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlrig
)
