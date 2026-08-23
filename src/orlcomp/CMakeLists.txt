set(ORLCOMP_LEXER_SOURCES
    orl_lexer.cpp
)

set(ORLCOMP_LEXER_HEADERS
    orl_lexer.h
)

set(ORLCOMP_PARSER_SOURCES
    orl_parser.cpp
    orl_preprocessor.cpp
)

set(ORLCOMP_PARSER_HEADERS
    orl_ast.h
    orl_parser.h
    orl_preprocessor.h
)

set(ORLCOMP_CODEGEN_SOURCES
    orl_codegen.cpp
    orl_intrinsics.cpp
    orl_optimizer.cpp
    orl_jit.cpp
    orl_gpu.cpp
    orl_parallel_runtime.cpp
)

set(ORLCOMP_CODEGEN_HEADERS
    orl_ast.h
    orl_codegen.h
    orl_intrinsics.h
    orl_optimizer.h
    orl_jit.h
    orl_gpu.h
    orl_parallel_runtime.h
    orl_runtime_signature.h
)

add_library(orlcomp_lexer
    ${ORLCOMP_LEXER_HEADERS}
    ${ORLCOMP_LEXER_SOURCES}
)

target_include_directories(orlcomp_lexer
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(orlcomp_parser
    ${ORLCOMP_PARSER_HEADERS}
    ${ORLCOMP_PARSER_SOURCES}
)

file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}/resource/stdlib" ORL_STDLIB_DIR)
target_include_directories(orlcomp_parser
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(orlcomp_parser
    PUBLIC "ORL_STDLIB_DIR=\"${ORL_STDLIB_DIR}\""
)

target_link_libraries(orlcomp_parser
    PUBLIC orlcomp_lexer
)

# vcpkg LLVM exports reference imported targets from these packages; load them
# with CONFIG mode before find_package(LLVM) processes LLVMExports.cmake.
find_package(ZLIB CONFIG QUIET)
find_package(zstd CONFIG QUIET)

find_package(LLVM CONFIG QUIET)

if (LLVM_FOUND)
    find_package(CUDAToolkit REQUIRED)
    find_package(TBB CONFIG REQUIRED)
    if (NOT TARGET ZLIB::ZLIB)
        message(FATAL_ERROR
            "LLVM was found but ZLIB::ZLIB is unavailable. "
            "Install zlib via vcpkg and pass -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake")
    endif()
    if (NOT TARGET zstd::libzstd_shared)
        message(FATAL_ERROR
            "LLVM was found but zstd::libzstd_shared is unavailable. "
            "Install zstd via vcpkg and pass -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake")
    endif()
    if (COMMAND llvm_map_components_to_libnames)
        set(ORLCOMP_LLVM_COMPONENTS
            Core Support ExecutionEngine
            OrcJIT OrcShared OrcTargetProcess
            Passes Analysis TransformUtils ScalarOpts InstCombine IPO AggressiveInstCombine
            X86CodeGen X86Desc X86Info X86AsmParser
        )
        if ("NVPTX" IN_LIST LLVM_TARGETS_TO_BUILD)
            list(APPEND ORLCOMP_LLVM_COMPONENTS
                NVPTXCodeGen NVPTXDesc NVPTXInfo
            )
        endif()
        llvm_map_components_to_libnames(ORLCOMP_LLVM_LIBS ${ORLCOMP_LLVM_COMPONENTS})
    else()
        set(ORLCOMP_LLVM_LIBS LLVM)
    endif()

    add_library(orlcomp_codegen
        ${ORLCOMP_CODEGEN_HEADERS}
        ${ORLCOMP_CODEGEN_SOURCES}
    )

    target_include_directories(orlcomp_codegen
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${LLVM_INCLUDE_DIRS}
        PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
    )

    target_compile_definitions(orlcomp_codegen PRIVATE ${LLVM_DEFINITIONS_LIST})

    target_link_libraries(orlcomp_codegen
        PUBLIC orlcomp_parser
        PUBLIC ${ORLCOMP_LLVM_LIBS}
               CUDA::cudart
        PRIVATE TBB::tbb
    )
    message(STATUS "Configured ORL LLVM IR codegen, optimizer, and JIT")
else()
    message(STATUS "LLVM not found; ORL LLVM IR codegen target disabled")
endif()

message(STATUS "Configured ORL compiler handwritten lexer and parser")
