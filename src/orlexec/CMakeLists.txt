if (NOT TARGET orlcomp_codegen)
    return()
endif()

add_library(orlexec
    orl_exec.cpp
    orl_exec.hpp
    orl_graph_exec.cpp
    orl_graph_exec.hpp
    orlrig/handle_registry.cpp
    orlrig/handle_registry.hpp
)

target_include_directories(orlexec
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(orlexec
    PUBLIC
        orlcomp_codegen
        orlcomp_graph_lowering
)

if(TARGET GLM::GLM)
    target_link_libraries(orlexec PUBLIC GLM::GLM)
elseif(TARGET glm::glm)
    target_link_libraries(orlexec PUBLIC glm::glm)
else()
    target_include_directories(orlexec PUBLIC "${GLM_INCLUDE_DIRS}")
endif()

add_executable(orlc
    ../orlcomp/orlc_main.cpp
)

target_link_libraries(orlc
    PRIVATE
        orlexec
        orlcomp_analysis
)

if (TARGET orlgraph_io)
    target_link_libraries(orlc PRIVATE orlgraph_io)
    target_compile_definitions(orlc PRIVATE ORL_HAS_GRAPH_IO=1)
endif()

add_library(orlrig
    orlrig/component_store.cpp
    orlrig/evaluation.cpp
    orlrig/evaluation.hpp
    orlrig/hierarchy.cpp
    orlrig/runners.cpp
    orlrig/abi.hpp
    orlrig/controller.hpp
    orlrig/deformer.hpp
    orlrig/graph_resources.cpp
    orlrig/graph_resources.hpp
    orlrig/hierarchy.hpp
    orlrig/ik.hpp
    orlrig/joint.hpp
    orlrig/locator.hpp
    orlrig/mesh.hpp
    orlrig/orlrig.hpp
    orlrig/runners.hpp
    orlrig/runner_status.hpp
    orlrig/two_stage_runner.cpp
    orlrig/two_stage_runner.hpp
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

if(TARGET TBB::tbb)
    target_link_libraries(orlrig PRIVATE TBB::tbb)
endif()

if(TARGET GLM::GLM)
    target_link_libraries(orlrig PUBLIC GLM::GLM)
elseif(TARGET glm::glm)
    target_link_libraries(orlrig PUBLIC glm::glm)
else()
    target_include_directories(orlrig PUBLIC "${GLM_INCLUDE_DIRS}")
endif()
