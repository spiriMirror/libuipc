# Create an extension module
pybind11_add_module(pyuipc)
target_compile_features(pyuipc PUBLIC cxx_std_20)
target_link_libraries(pyuipc PUBLIC pybind11::module)
target_link_libraries(pyuipc PUBLIC uipc::uipc)
# ensure all backends are built before pyuipc
add_dependencies(pyuipc uipc::backends)
target_include_directories(pyuipc PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../")
target_compile_definitions(pyuipc PRIVATE "PYBIND11_DETAILED_ERROR_MESSAGES")
uipc_target_set_output_directory(pyuipc)
uipc_target_set_rpath(pyuipc)

file(GLOB SOURCES "*.cpp" "*.h")
target_sources(pyuipc PRIVATE ${SOURCES})

add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(geometry)
add_subdirectory(constitution)
add_subdirectory(backend)
add_subdirectory(builtin)
add_subdirectory(diff_sim)

if (UIPC_WITH_USD_SUPPORT)
    target_compile_definitions(pyuipc PRIVATE "UIPC_WITH_USD_SUPPORT=1")
    add_subdirectory(usd)
endif()

uipc_target_set_relative_source_file(pyuipc)

# Setup IDE
get_target_property(SOURCES pyuipc SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})

uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH} "mypy")
uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH}  "numpy")

# After build pyuipc, call the script to copy the dependent shared libraries to python package.
set(UIPC_PYTHON_STUB_OUTPUT_DIR "${CMAKE_BINARY_DIR}/python/src")

if(NOT UIPC_BUILD_PYTHON_WHEEL)
    add_custom_command(TARGET pyuipc POST_BUILD
        COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} 
        ARGS
        "${PROJECT_SOURCE_DIR}/scripts/after_build_pyuipc.py"
        "--target=$<TARGET_FILE:pyuipc>"
        "--binary_dir=${CMAKE_BINARY_DIR}"
        "--config=$<CONFIG>"
        "--build_type=${CMAKE_BUILD_TYPE}"
        "--stub-output=${UIPC_PYTHON_STUB_OUTPUT_DIR}"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Copying pyuipc dependent shared libraries to python package"
    )
else()
    add_custom_command(TARGET pyuipc POST_BUILD
        COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} 
        ARGS
        "${PROJECT_SOURCE_DIR}/scripts/generate_stubs.py"
        "--target-dir=${CMAKE_BINARY_DIR}/python/src/uipc"
        "--output-dir=${UIPC_PYTHON_STUB_OUTPUT_DIR}"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Generating stubs for pyuipc"
    )
    install(DIRECTORY "${UIPC_PYTHON_STUB_OUTPUT_DIR}/pyuipc/"
        DESTINATION pyuipc
        FILES_MATCHING PATTERN "*.pyi"
    )
endif()


# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
uipc_install_target(pyuipc)

# Install all vcpkg runtime dependencies (DLLs/shared libraries)
# This installs DLLs from vcpkg_installed/bin which contain dependencies like spdlog, fmt, TBB, etc.
uipc_install_vcpkg_runtime()

