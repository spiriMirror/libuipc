find_package(Python REQUIRED QUIET)
find_package (Python COMPONENTS Interpreter Development REQUIRED QUIET)
find_package(pybind11 CONFIG REQUIRED QUIET)


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

# Setup IDE
get_target_property(SOURCES pyuipc SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})

uipc_info("Python found: ${Python_EXECUTABLE}.")
message(STATUS "  > Make sure that the following dependencies are installed in the same environment:")
message(STATUS "    * mypy  -> we use `mypy.stubgen` to generate the stub files for `pyuipc`.")
message(STATUS "    * numpy -> dependency for `pyuipc`.")



# after build pyuipc, call the script to copy the dependent shared libraries to python package
add_custom_command(TARGET pyuipc POST_BUILD
    COMMAND ${Python_EXECUTABLE} 
    ARGS
    "${PROJECT_SOURCE_DIR}/scripts/after_build_pyuipc.py"
    "--target=$<TARGET_FILE:pyuipc>"
    "--binary_dir=${CMAKE_BINARY_DIR}"
    "--config=$<CONFIG>"
    "--build_type=${CMAKE_BUILD_TYPE}"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copying pyuipc dependent shared libraries to python package"
)

