find_package (Python COMPONENTS Interpreter Development REQUIRED QUIET)
find_package(pybind11 CONFIG REQUIRED QUIET)


# Create an extension module
pybind11_add_module(pyuipc)
target_compile_features(pyuipc PUBLIC cxx_std_20)
target_link_libraries(pyuipc PUBLIC pybind11::module)
target_link_libraries(pyuipc PUBLIC uipc::uipc)
uipc_set_output_directory(pyuipc)

file(GLOB SOURCES "*.cpp" "*.h")
target_sources(pyuipc PRIVATE ${SOURCES})

add_subdirectory(common)
add_subdirectory(engine)
add_subdirectory(geometry)
add_subdirectory(world)
add_subdirectory(constitution)
add_subdirectory(backend)
add_subdirectory(builtin)

# Setup IDE
get_target_property(SOURCES pyuipc SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})

# after build pyuipc, call copy.py to copy the Release
add_custom_command(TARGET pyuipc POST_BUILD
    COMMAND ${Python_EXECUTABLE} ${CMAKE_PROJECT_SOURCE_DIR}/scripts/copy_release.py
    WORKING_DIRECTORY ${CMAKE_TARGET_OUTPUT_DIRECTORY}
    COMMENT "Copying Release to source/python/bin/..."
)

