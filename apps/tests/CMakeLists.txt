find_package(Catch2 CONFIG REQUIRED)

get_filename_component(test_main_cpp "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" ABSOLUTE)

# -----------------------------------------------------------------------------------
# function to add a test
# -----------------------------------------------------------------------------------
function(uipc_add_test name)
    add_executable(${name} ${ARGN})
    target_sources(${name} PRIVATE ${test_main_cpp})
    target_link_libraries(${name} PRIVATE 
        uipc::uipc
        app
        Catch2::Catch2 
        Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_test_${name}")

    # add a dependency to the backends, so that the backends can be updated before the tests are run
    uipc_add_backend_dependency(${name})
    # set the output directory to keep the binary files in the same directory
    uipc_set_output_directory(${name})
endfunction()

function(uipc_add_gui_test name)
    uipc_add_test(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE uipc::gui)
endfunction()

add_subdirectory(geometry)
add_subdirectory(world)
add_subdirectory(common)
add_subdirectory(backends)

if(UIPC_BUILD_GUI)
    add_subdirectory(gui)
endif()