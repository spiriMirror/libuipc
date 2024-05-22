find_package(Catch2 CONFIG REQUIRED)

get_filename_component(test_main_cpp "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" ABSOLUTE)

# function to add a test
function(uipc_add_test name source)
    add_executable(${name} "${source}")
    target_sources(${name} PRIVATE ${test_main_cpp})
    target_link_libraries(${name} PRIVATE uipc Catch2::Catch2 Catch2::Catch2WithMain)
    set_property(TARGET ${name} PROPERTY FOLDER "tests")
endfunction()

add_subdirectory(geometry)