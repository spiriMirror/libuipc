find_package(Catch2 CONFIG REQUIRED)

get_filename_component(test_main_cpp "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" ABSOLUTE)

# function to add a test
function(uipc_add_test name)
    add_executable(${name} ${ARGN})
    target_sources(${name} PRIVATE ${test_main_cpp})
    target_link_libraries(${name} PRIVATE uipc app Catch2::Catch2 Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "apps/tests")
endfunction()

add_subdirectory(geometry)