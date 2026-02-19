find_package(Catch2 CONFIG REQUIRED)

# -----------------------------------------------------------------------------------
# function to add a test
# -----------------------------------------------------------------------------------
function(uipc_executable_as_test name)
    target_link_libraries(${name} PRIVATE
            app
            uipc::uipc
            uipc::backend::none
            Catch2::Catch2)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_test_${name}")
    uipc_target_add_include_files(${name})
    # set the output directory to keep the binary files in the same directory
    uipc_target_set_output_directory(${name})
endfunction()

set(UIPC_TEST_MAIN ${CMAKE_CURRENT_LIST_DIR}/catch_main.cpp)

function(uipc_add_test name)
    add_executable(${name} ${UIPC_TEST_MAIN} ${ARGN})
    uipc_target_set_relative_source_file(${name})
    uipc_executable_as_test(${name})
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

if(UIPC_WITH_USD_SUPPORT)
    add_subdirectory(usd)
endif()