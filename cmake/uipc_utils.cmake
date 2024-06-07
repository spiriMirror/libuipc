# -----------------------------------------------------------------------------------------
# Set the output directory for the target
# -----------------------------------------------------------------------------------------
function(uipc_set_output_directory target_name)
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/debug)
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/release)
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/bin/relwithdebinfo)

    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/debug)
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/release)
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/bin/relwithdebinfo)

    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib/debug)
    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib/release)
    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/lib/relwithdebinfo)
endfunction()

# -----------------------------------------------------------------------------------------
# print message info with uipc prefix
# -----------------------------------------------------------------------------------------
function(uipc_info content)
    message(STATUS "[libuipc]: ${content}")
endfunction()

# -----------------------------------------------------------------------------------------
# Add a dependency to the backends, so that the backends will be built before this target
# -----------------------------------------------------------------------------------------
function(uipc_add_backend_dependency target_name)
    add_dependencies(${target_name} uipc::backends)
endfunction()



