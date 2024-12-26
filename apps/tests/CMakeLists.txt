find_package(Catch2 CONFIG REQUIRED)

# -----------------------------------------------------------------------------------
# function to add a test
# -----------------------------------------------------------------------------------
function(uipc_add_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE
        app
        Catch2::Catch2 
        Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_test_${name}")
    uipc_target_add_include_files(${name})
    # add a dependency to the backends, so that the backends can be updated before the tests are run
    uipc_target_add_backend_dependency(${name})
    # set the output directory to keep the binary files in the same directory
    uipc_target_set_output_directory(${name})
endfunction()

function(uipc_add_gui_test name)
    uipc_add_test(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE uipc::gui)
endfunction()

add_subdirectory(geometry)
add_subdirectory(core)
add_subdirectory(common)
add_subdirectory(backends)
add_subdirectory(sim_case)
add_subdirectory(sanity_check)

if(UIPC_BUILD_GUI)
    uipc_warning("This version C++ GUI is not finished yet, waiting for contributions.")
    # add_subdirectory(gui)
endif()