cmake_minimum_required(VERSION 3.20)

project(uipc)

# =========================================================================
option(UIPC_CORE_ONLY "Build only the core library" OFF)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_EXAMPLES "Build examples" ON)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" ON)
option(UIPC_WITH_CUDA_BACKEND "Build with CUDA backend" ON)
# =========================================================================

# set cmake include path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# submodule dependencies
add_subdirectory(external/rapidcsv)

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

