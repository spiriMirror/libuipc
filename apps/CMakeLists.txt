
add_subdirectory(app)

# applications
if(UIPC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# tests
if(UIPC_BUILD_TESTS)
    add_subdirectory(tests)
endif()