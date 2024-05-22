cmake_minimum_required(VERSION 3.20)

project(uipc)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(src/uipc)

add_subdirectory(examples)

option(UIPC_BUILD_TESTS "Build tests" ON)

if(UIPC_BUILD_TESTS)
    add_subdirectory(tests)
endif()