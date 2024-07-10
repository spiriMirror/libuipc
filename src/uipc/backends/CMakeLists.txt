include(uipc_utils)
file(GLOB BACKEND_COMMON_SOURCES "*.h" "*.cpp" "details/*.inl")


# ---------------------------------------------------------------------------
# A dummy target to group all backends
# ---------------------------------------------------------------------------
add_library(uipc_backends INTERFACE)
add_library(uipc::backends ALIAS uipc_backends)


# ---------------------------------------------------------------------------
# function to set some common properties for backends
# ---------------------------------------------------------------------------
function(uipc_add_backend name)
    uipc_info("Adding backend: [${name}]")

    if(UIPC_BUILD_TESTS) # for tests, we need to link against the shared library
        # uipc_info("UIPC_BUILD_TESTS=${UIPC_BUILD_TESTS}, so we build the backend [${name}] as shared library")
        add_library(${name} SHARED)
    else() # else, we just dynamically load the backend
        add_library(${name} MODULE)
    endif()

    # alias the target
    add_library(uipc::backend::${name} ALIAS ${name})

    target_link_libraries(${name} PRIVATE uipc::uipc)
    target_compile_definitions(${name} PRIVATE UIPC_BACKEND_EXPORT_DLL=1)
    target_compile_features(${name} PRIVATE cxx_std_20)

    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_backend_${name}")
    
    # folder for IDE
    set_target_properties(${name} PROPERTIES FOLDER "uipc-backends")
    target_sources(${name} PRIVATE ${BACKEND_COMMON_SOURCES})

    uipc_set_output_directory(${name})
    add_dependencies(uipc_backends ${name})
endfunction()


# ---------------------------------------------------------------------------
# Add backends
# ---------------------------------------------------------------------------
add_subdirectory(none)
add_subdirectory(cuda)




