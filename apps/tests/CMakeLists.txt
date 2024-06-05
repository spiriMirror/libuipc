find_package(Catch2 CONFIG REQUIRED)

get_filename_component(test_main_cpp "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" ABSOLUTE)

# function to add a test
function(uipc_add_test name)
    add_executable(${name} ${ARGN})
    target_sources(${name} PRIVATE ${test_main_cpp})
    target_link_libraries(${name} PRIVATE uipc::uipc app Catch2::Catch2 Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
    uipc_set_output_directory(${name})
    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_test_${name}")
endfunction()

function(uipc_add_gui_test name)
    add_executable(${name} ${ARGN})
    target_sources(${name} PRIVATE ${test_main_cpp})
    target_link_libraries(${name} PRIVATE uipc_gui app Catch2::Catch2 Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
    uipc_set_output_directory(${name})
endfunction()

add_subdirectory(geometry)
add_subdirectory(world)
add_subdirectory(common)
add_subdirectory(backend)

if(UIPC_BUILD_GUI)
    add_subdirectory(gui)
endif()