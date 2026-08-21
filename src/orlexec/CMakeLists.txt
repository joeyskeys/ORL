if (NOT TARGET orlcomp_codegen)
    return()
endif()

add_library(orlexec
    orl_exec.cpp
    orl_exec.hpp
)

target_include_directories(orlexec
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(orlexec
    PUBLIC
        orlcomp_codegen
)
