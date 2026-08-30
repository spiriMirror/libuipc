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

    add_test(NAME uipc.${name} COMMAND $<TARGET_FILE:${name}>)
    set_tests_properties(uipc.${name} PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        TIMEOUT 900)

    if(name STREQUAL "common" OR name STREQUAL "core" OR name STREQUAL "geometry"
       OR (name STREQUAL "sanity_check" AND NOT UIPC_WITH_CUDA_BACKEND))
        set_tests_properties(uipc.${name} PROPERTIES LABELS "fast;cpu")
    else()
        set_tests_properties(uipc.${name} PROPERTIES LABELS "gpu")
    endif()
endfunction()

add_subdirectory(geometry)
add_subdirectory(core)
add_subdirectory(common)
add_subdirectory(sanity_check)

if(UIPC_WITH_CUDA_BACKEND)
    add_subdirectory(backends)
    add_subdirectory(sim_case)
    add_subdirectory(regression)
endif()

if(UIPC_WITH_USD_SUPPORT)
    add_subdirectory(usd)
endif()
