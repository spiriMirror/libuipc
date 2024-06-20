# -----------------------------------------------------------------------------------------
# Libuipc Logo
# -----------------------------------------------------------------------------------------
function(uipc_show_logo version)
message(STATUS "
-----------------------------------------------------------------------------------
                                   v.${version}
                ██      ██ ██████  ██    ██ ██ ██████   ██████
                ██      ██ ██   ██ ██    ██ ██ ██   ██ ██     
                ██      ██ ██████  ██    ██ ██ ██████  ██     
                ██      ██ ██   ██ ██    ██ ██ ██      ██     
                ███████ ██ ██████   ██████  ██ ██       ██████
        LIBUIPC: A C++20 Unified Incremental Potentional Contact Library
-----------------------------------------------------------------------------------")
endfunction()

# -----------------------------------------------------------------------------------------
# Print message info with uipc prefix
# -----------------------------------------------------------------------------------------
function(uipc_info content)
    message(STATUS "[libuipc] ${content}")
endfunction()

# -----------------------------------------------------------------------------------------
# Print message warning with uipc prefix
# -----------------------------------------------------------------------------------------
function(uipc_warning content)
    message(WARNING "[libuipc] ${content}")
endfunction()

# -----------------------------------------------------------------------------------------
# Print message error with uipc prefix
# -----------------------------------------------------------------------------------------
function(uipc_error content)
    message(FATAL_ERROR "[libuipc] ${content}")
endfunction()

# -----------------------------------------------------------------------------------------
# Print message fatal error with uipc prefix
# -----------------------------------------------------------------------------------------
function(uipc_fatal_error content)
    message(FATAL_ERROR "[libuipc] ${content}")
endfunction()

# -----------------------------------------------------------------------------------------
# Print the options of the project
# -----------------------------------------------------------------------------------------
function(uipc_show_options)
    uipc_info("Options:")
    message(STATUS "    * UIPC_CORE_ONLY: ${UIPC_CORE_ONLY}")
    message(STATUS "    * UIPC_BUILD_GUI: ${UIPC_BUILD_GUI}")
    message(STATUS "    * UIPC_BUILD_EXAMPLES: ${UIPC_BUILD_EXAMPLES}")
    message(STATUS "    * UIPC_BUILD_TESTS: ${UIPC_BUILD_TESTS}")
    message(STATUS "    * UIPC_BUILD_BENCHMARKS: ${UIPC_BUILD_BENCHMARKS}")
    message(STATUS "    * UIPC_WITH_CUDA_BACKEND: ${UIPC_WITH_CUDA_BACKEND}")
endfunction()

# -----------------------------------------------------------------------------------------
# Generate the vcpkg manifest file, called only once, at the every beginning of the build
# -----------------------------------------------------------------------------------------
function(uipc_generate_vcpkg_manifest)
    find_package(Python REQUIRED)

    # call python script to generate vcpkg.json, pass the CMAKE_BINARY_DIR as argument
    execute_process(
        COMMAND ${Python_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_vcpkg_json.py"
        ${CMAKE_BINARY_DIR} # pass the CMAKE_BINARY_DIR as argument
        "--build_gui=${UIPC_BUILD_GUI}" # pass the UIPC_BUILD_GUI as argument
    )

    set(VCPKG_MANIFEST_DIR "${CMAKE_BINARY_DIR}" PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------------------
# Install the target to the correct directory
# -----------------------------------------------------------------------------------------
function(uipc_install target_name)
    # This function is mainly for linux system to install the target to the correct directory
    # On windows, the `uipc_set_output_directory()` is enough to set the output directory
    # Put Debug/Release/RelWithDebInfo into different directories
    install(TARGETS ${target_name}
        CONFIGURATIONS Debug
        RUNTIME DESTINATION "Debug/bin"
        LIBRARY DESTINATION "Debug/bin"
        ARCHIVE DESTINATION "Debug/lib"
    )
    install(TARGETS ${target_name}
        CONFIGURATIONS Release
        RUNTIME DESTINATION "Release/bin"
        LIBRARY DESTINATION "Release/bin"
        ARCHIVE DESTINATION "Release/lib"
    )
    install(TARGETS ${target_name}
        CONFIGURATIONS RelWithDebInfo
        RUNTIME DESTINATION "RelWithDebInfo/bin"
        LIBRARY DESTINATION "RelWithDebInfo/bin"
        ARCHIVE DESTINATION "RelWithDebInfo/lib"
    )
endfunction()

# -----------------------------------------------------------------------------------------
# Set the output directory for the target
# -----------------------------------------------------------------------------------------
function(uipc_set_output_directory target_name)
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/bin")
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/bin")
    set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/bin")

    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/bin")
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/bin")
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/bin")

    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/lib")
    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/lib")
    set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/lib")

    uipc_install(${target_name})
endfunction()

# -----------------------------------------------------------------------------------------
# Add a dependency to the backends, so that the backends will be built before this target
# to make sure the backends are always up-to-date when developing the target
# -----------------------------------------------------------------------------------------
function(uipc_add_backend_dependency target_name)
    add_dependencies(${target_name} uipc::backends)
endfunction()



