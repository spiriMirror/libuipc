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
add_subdirectory(engine)
add_subdirectory(geometry)
add_subdirectory(world)
add_subdirectory(constitution)
add_subdirectory(backend)
add_subdirectory(builtin)

# Setup IDE
get_target_property(SOURCES pyuipc SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})


# copy the target shared library to python package
add_custom_command(TARGET pyuipc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:pyuipc>
    "${PROJECT_SOURCE_DIR}/python/src/pyuipc_loader/$<CONFIG>/bin/"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# after build pyuipc, call copy_release.py to copy shared library to python package
add_custom_command(TARGET pyuipc POST_BUILD
    COMMAND ${Python_EXECUTABLE} 
    ARGS
    "${PROJECT_SOURCE_DIR}/scripts/after_build_pyuipc.py"
    "--dir=${CMAKE_BINARY_DIR}/$<CONFIG>/bin"
    "--config=$<CONFIG>"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copying pyuipc dependent shared libraries to python package"
)

