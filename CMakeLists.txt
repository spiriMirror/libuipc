cmake_minimum_required(VERSION 3.20)

project(uipc)

# =========================================================================
option(UIPC_CORE_ONLY "Build only the core library" OFF)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_EXAMPLES "Build examples" ON)
# =========================================================================


set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# submodule dependencies
add_subdirectory(external/enumerate)
add_subdirectory(external/rapidcsv)

# uipc
add_subdirectory(src/uipc)

# if only core is needed, ignore the rest
if (UIPC_CORE_ONLY) 
    return()
endif()

# applications
add_subdirectory(apps)