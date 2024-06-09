# common header for applications
add_subdirectory(app)

# examples
if(UIPC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# tests
if(UIPC_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# benchmarks
if(UIPC_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()