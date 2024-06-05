include(uipc_utils)
set(MODULE_FUNCTION_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/module.h")

function(uipc_add_backend name)
    add_library(${name} MODULE ${ARGN})
    target_link_libraries(${name} PRIVATE uipc::uipc)
    target_compile_definitions(${name} PRIVATE UIPC_BACKEND_EXPORT_DLL=1)
    # alias the target
    add_library(uipc::backend::${name} ALIAS ${name})
    set_target_properties(${name} PROPERTIES
        PREFIX ""
        OUTPUT_NAME "uipc_backend_${name}"
    )
    # folder for IDE
    set_target_properties(${name} PROPERTIES FOLDER "uipc-backends")
    target_sources(${name} PRIVATE ${MODULE_FUNCTION_HEADER})

    uipc_set_output_directory(${name})
endfunction()

add_subdirectory(none)

# Group files in IDE
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCE_GROUP_FILES})

