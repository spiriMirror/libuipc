# Create an extension module
pybind11_add_module(pyuipc)
target_compile_features(pyuipc PUBLIC cxx_std_20)
target_link_libraries(pyuipc PUBLIC pybind11::module)
target_link_libraries(pyuipc PUBLIC uipc::uipc)
target_include_directories(pyuipc PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../")
uipc_target_set_output_directory(pyuipc)

file(GLOB SOURCES "*.cpp" "*.h")
target_sources(pyuipc PRIVATE ${SOURCES})

add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(geometry)
add_subdirectory(constitution)
add_subdirectory(backend)
add_subdirectory(builtin)
add_subdirectory(diff_sim)

# Setup IDE
get_target_property(SOURCES pyuipc SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})

uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH} "mypy")
uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH}  "numpy")

# after build pyuipc, call the script to copy the dependent shared libraries to python package
add_custom_command(TARGET pyuipc POST_BUILD
    COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} 
    ARGS
    "${PROJECT_SOURCE_DIR}/scripts/after_build_pyuipc.py"
    "--target=$<TARGET_FILE:pyuipc>"
    "--binary_dir=${CMAKE_BINARY_DIR}"
    "--config=$<CONFIG>"
    "--build_type=${CMAKE_BUILD_TYPE}"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copying pyuipc dependent shared libraries to python package"
)

