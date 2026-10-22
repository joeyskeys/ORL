add_library(orl_rhi_contracts INTERFACE)
target_include_directories(orl_rhi_contracts
    INTERFACE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_BINARY_DIR}/generated"
)
target_compile_features(orl_rhi_contracts INTERFACE cxx_std_20)

add_executable(orl_rhi_contract_tests
    rhi_contract_tests.cpp
)
target_link_libraries(orl_rhi_contract_tests
    PRIVATE
        orl_rhi_contracts
)

if(NOT ORL_BUILD_RHI_ADAPTERS)
    return()
endif()

if(ORL_RHI_BACKEND_RESOLVED STREQUAL "VKKK")
    if(NOT TARGET vkkk)
        message(FATAL_ERROR
            "The selected VKKK RHI adapter requires the vkkk target")
    endif()

    add_library(orl_rhi_vkkk STATIC
        vkkk_adapter.cpp
        vkkk_adapter.hpp
        vkkk_adapter_impl.hpp
        vkkk_display.cpp
        vkkk_native.hpp
    )
    target_include_directories(orl_rhi_vkkk
        PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}"
        # vkkk_native.hpp exposes vkkk types to the legacy viewer features.
        PUBLIC
            "${CMAKE_SOURCE_DIR}/thirdparty/vkkk/src"
    )
    target_link_libraries(orl_rhi_vkkk
        PUBLIC
            orl_rhi_contracts
            vkkk
    )
    target_compile_features(orl_rhi_vkkk PUBLIC cxx_std_20)
    if(ORL_USE_QT6)
        target_compile_definitions(orl_rhi_vkkk PUBLIC ORL_USE_QT6=1)
    else()
        target_compile_definitions(orl_rhi_vkkk PUBLIC ORL_USE_QT6=0)
    endif()
    add_library(orl_rhi_selected ALIAS orl_rhi_vkkk)
elseif(ORL_RHI_BACKEND_RESOLVED STREQUAL "MT")
    if(NOT TARGET mt::mt)
        message(FATAL_ERROR
            "The selected MT RHI adapter requires mt::mt")
    endif()

    add_library(orl_rhi_mt STATIC
        mt_adapter.cpp
        mt_adapter.hpp
        mt_display.cpp
    )
    target_include_directories(orl_rhi_mt
        PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(orl_rhi_mt
        PUBLIC
            orl_rhi_contracts
            mt::mt
    )
    target_compile_features(orl_rhi_mt PUBLIC cxx_std_20)
    add_library(orl_rhi_selected ALIAS orl_rhi_mt)
endif()

if(TARGET orl_rhi_selected)
    add_executable(orl_rhi_selected_compile
        rhi_selected_compile.cpp
    )
    target_link_libraries(orl_rhi_selected_compile
        PRIVATE
            orl_rhi_selected
    )
endif()
