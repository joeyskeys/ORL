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

add_library(orlrig
    orlrig/component_store.cpp
    orlrig/runners.cpp
    orlrig/abi.hpp
    orlrig/controller.hpp
    orlrig/deformer.hpp
    orlrig/ik.hpp
    orlrig/joint.hpp
    orlrig/mesh.hpp
    orlrig/orlrig.hpp
    orlrig/runners.hpp
    orlrig/weight.hpp
    orlrig/component_store.hpp
)

target_include_directories(orlrig
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(orlrig
    PUBLIC
        orlexec
)

if(TARGET GLM::GLM)
    target_link_libraries(orlrig PUBLIC GLM::GLM)
elseif(TARGET glm::glm)
    target_link_libraries(orlrig PUBLIC glm::glm)
else()
    target_include_directories(orlrig PUBLIC "${GLM_INCLUDE_DIRS}")
endif()
