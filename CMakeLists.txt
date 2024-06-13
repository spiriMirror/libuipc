cmake_minimum_required(VERSION 3.27)

# =========================================================================
option(UIPC_CORE_ONLY "Build only the core library" OFF)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" ON)

option(UIPC_BUILD_EXAMPLES "Build examples" ON)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_BENCHMARKS "Build benchmarks" ON)

option(UIPC_WITH_CUDA_BACKEND "Build with CUDA backend" ON)
# =========================================================================

include(cmake/uipc_utils.cmake)
# generate the dependency manifest according to the options
uipc_generate_vcpkg_manifest()

project(uipc)
message(STATUS "-----------------------------------------------------------------------------------")
uipc_info("Options:")
message(STATUS "    * UIPC_CORE_ONLY: ${UIPC_CORE_ONLY}")
message(STATUS "    * UIPC_BUILD_GUI: ${UIPC_BUILD_GUI}")
message(STATUS "    * UIPC_BUILD_EXAMPLES: ${UIPC_BUILD_EXAMPLES}")
message(STATUS "    * UIPC_BUILD_TESTS: ${UIPC_BUILD_TESTS}")
message(STATUS "    * UIPC_BUILD_BENCHMARKS: ${UIPC_BUILD_BENCHMARKS}")
message(STATUS "    * UIPC_WITH_CUDA_BACKEND: ${UIPC_WITH_CUDA_BACKEND}")

# set cmake include path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# submodule dependencies
# add_subdirectory(external/rapidcsv)

if(UIPC_WITH_CUDA_BACKEND)
    enable_language(CUDA)
    # set options for cuda
    set(MUDA_BUILD_EXAMPLE OFF CACHE BOOL "Build examples" FORCE)
    add_subdirectory(external/muda)
endif()

# uipc
add_subdirectory(src/uipc)

# if only core is needed, ignore the rest
if (UIPC_CORE_ONLY) 
    return()
endif()

if(UIPC_BUILD_GUI)
    # gui
    add_subdirectory(src/uipc_gui)
endif()

# applications
add_subdirectory(apps)

message(STATUS "-----------------------------------------------------------------------------------")
