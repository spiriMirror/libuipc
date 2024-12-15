find_package(Catch2 CONFIG REQUIRED)

# get_filename_component(benchmark_main_cpp "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" ABSOLUTE)

function(uipc_add_benchmark name)
    add_executable(${name} ${ARGN})
    target_sources(${name} PRIVATE ${benchmark_main_cpp})
    target_link_libraries(${name} PRIVATE 
        uipc::uipc
        app
        Catch2::Catch2
        Catch2::Catch2WithMain)

    set_property(TARGET ${name} PROPERTY FOLDER "apps/benchmarks")
    set_target_properties(${name} PROPERTIES OUTPUT_NAME "uipc_benchmark_${name}")

    # add a dependency to the backends, so that the backends can be updated before the tests are run
    uipc_target_add_backend_dependency(${name})
    # set the output directory to keep the binary files in the same directory
    uipc_target_set_output_directory(${name})
endfunction()

add_subdirectory(basic)