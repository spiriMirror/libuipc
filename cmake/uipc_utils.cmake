# -----------------------------------------------------------------------------------------
# Libuipc Logo
# -----------------------------------------------------------------------------------------
function(uipc_show_logo)
set(M ${UIPC_VERSION_MAJOR})
set(m ${UIPC_VERSION_MINOR})
set(p ${UIPC_VERSION_PATCH})
message(STATUS "
-----------------------------------------------------------------------------------
                                   v ${M}.${m}.${p}
                ██      ██ ██████  ██    ██ ██ ██████   ██████
                ██      ██ ██   ██ ██    ██ ██ ██   ██ ██     
                ██      ██ ██████  ██    ██ ██ ██████  ██     
                ██      ██ ██   ██ ██    ██ ██ ██      ██     
                ███████ ██ ██████   ██████  ██ ██       ██████
        LIBUIPC: A C++20 Unified Incremental Potentional Contact Library
-----------------------------------------------------------------------------------")
endfunction()

# -----------------------------------------------------------------------------------------
# Print message info with uipc prefix
# -----------------------------------------------------------------------------------------
macro(uipc_info content)
    message(STATUS "[libuipc] ${content}")
endmacro()

# -----------------------------------------------------------------------------------------
# Print message warning with uipc prefix
# -----------------------------------------------------------------------------------------
macro(uipc_warning content)
    message(WARNING "[libuipc] ${content}")
endmacro()

# -----------------------------------------------------------------------------------------
# Print message error with uipc prefix
# -----------------------------------------------------------------------------------------
macro(uipc_error content)
    message(FATAL_ERROR "[libuipc] ${content}")
endmacro()

# -----------------------------------------------------------------------------------------
# Print the options of the project
# -----------------------------------------------------------------------------------------
function(uipc_show_options)
    uipc_info("Options:")
    message(STATUS "    * UIPC_DEV_MODE: ${UIPC_DEV_MODE}")
    message(STATUS "    * UIPC_BUILD_GUI: ${UIPC_BUILD_GUI}")
    message(STATUS "    * UIPC_BUILD_PYBIND: ${UIPC_BUILD_PYBIND}")
    message(STATUS "    * UIPC_BUILD_TORCH_EXTENSION: ${UIPC_BUILD_TORCH_EXTENSION}")
    message(STATUS "    * UIPC_USING_LOCAL_VCPKG: ${UIPC_USING_LOCAL_VCPKG}")
    message(STATUS "    * UIPC_BUILD_EXAMPLES: ${UIPC_BUILD_EXAMPLES}")
    message(STATUS "    * UIPC_BUILD_TESTS: ${UIPC_BUILD_TESTS}")
    message(STATUS "    * UIPC_BUILD_BENCHMARKS: ${UIPC_BUILD_BENCHMARKS}")
    message(STATUS "    * UIPC_WITH_CUDA_BACKEND: ${UIPC_WITH_CUDA_BACKEND}")
    message(STATUS "    * UIPC_PYTHON_EXECUTABLE_PATH: ${UIPC_PYTHON_EXECUTABLE_PATH}")
endfunction()

# -----------------------------------------------------------------------------------------
# Full path of the python executable
# -----------------------------------------------------------------------------------------
function(uipc_find_python_executable_path)
    if ("${UIPC_PYTHON_EXECUTABLE_PATH}" STREQUAL "")
        find_package(Python REQUIRED QUIET)
        # find_package (Python COMPONENTS Interpreter Development REQUIRED QUIET)
        if(NOT Python_FOUND)
            uipc_error("Python is required to generate vcpkg.json. Please install Python.")
        endif()
        # set the python executable path cache
        set(UIPC_PYTHON_EXECUTABLE_PATH "${Python_EXECUTABLE}" CACHE STRING "Python executable path" FORCE)
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Config the vcpkg install: a fast build bootstrap to avoid the long time vcpkg package
# checking. Only check and install the packages first time.
# -----------------------------------------------------------------------------------------
function(uipc_config_vcpkg_install)
    set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    set(VCPKG_MANIFEST_FILE "${VCPKG_MANIFEST_DIR}/vcpkg.json")
    if ("${CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
        uipc_error(
        "`CMAKE_TOOLCHAIN_FILE` is not set. It seems that CMake can't find the Vcpkg\n"
        "Please setup the environment variable `CMAKE_TOOLCHAIN_FILE` to your vcpkg.cmake file.\n" 
        "Details: https://spirimirror.github.io/libuipc-doc/build_install/")
    endif()
    file(TO_CMAKE_PATH "${CMAKE_TOOLCHAIN_FILE}" CMAKE_TOOLCHAIN_FILE)
    uipc_info("CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
    uipc_find_python_executable_path()
    # call python script to generate vcpkg.json, pass the CMAKE_BINARY_DIR as argument
    execute_process(
        COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_vcpkg_json.py"
        ${VCPKG_MANIFEST_DIR} # pass the CMAKE_CURRENT_BINARY_DIR as vcpkg.json output directory
        "--build_gui=${UIPC_BUILD_GUI}" # pass the UIPC_BUILD_GUI as argument
        "--dev_mode=${UIPC_DEV_MODE}" # pass the UIPC_DEV_MODE as argument
        RESULT_VARIABLE VCPKG_JSON_GENERATE_RESULT # return code 1 for need install, 0 for no need install
    )

    # set VCPKG_MANIFEST_INSTALL option to control the vcpkg install
    if(VCPKG_JSON_GENERATE_RESULT)
        set(VCPKG_MANIFEST_INSTALL ON CACHE BOOL "" FORCE)
    else()
        set(VCPKG_MANIFEST_INSTALL OFF CACHE BOOL "" FORCE)
    endif()
    if(UIPC_GITHUB_ACTIONS)
        set(VCPKG_MANIFEST_INSTALL ON CACHE BOOL "" FORCE)
    endif()
    # message(STATUS "VCPKG_MANIFEST_INSTALL: ${VCPKG_MANIFEST_INSTALL}")

    set(VCPKG_INSTALLED_DIR "")
    if(UIPC_USING_LOCAL_VCPKG)
        set(VCPKG_INSTALLED_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed")
    else()
        if (DEFINED $ENV{VCPKG_ROOT})
            set(VCPKG_INSTALLED_DIR "$ENV{VCPKG_ROOT}/installed")
        else()
            uipc_error("When using system vcpkg (UIPC_USING_LOCAL_VCPKG=${UIPC_USING_LOCAL_VCPKG}), please set the VCPKG_ROOT environment variable to the vcpkg root directory.")
        endif()
    endif()
    
    uipc_info("Package install directory: ${VCPKG_INSTALLED_DIR}")

    # export some variables to the parent scope
    set(VCPKG_MANIFEST_DIR "${VCPKG_MANIFEST_DIR}" PARENT_SCOPE)
    # set(VCPKG_TRACE_FIND_PACKAGE ON PARENT_SCOPE)
    set(VCPKG_INSTALLED_DIR "${VCPKG_INSTALLED_DIR}" PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------------------
# Set the output directory for the target
# -----------------------------------------------------------------------------------------
function(uipc_target_set_output_directory target_name)
    if(WIN32) # if on windows, set the output directory with different configurations
        
        set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/bin")
        set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/bin")
        set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/bin")

        set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/bin")
        set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/bin")
        set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/bin")

        set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/lib")
        set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/lib")
        set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo/lib")
    elseif(UNIX)  # if on linux, set the output directory
        if("${CMAKE_BUILD_TYPE}" STREQUAL "") # if the build type is not set, set it to Release
            set(CMAKE_BUILD_TYPE "Release")
        endif()
        set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin")
        set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin")
        set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib")
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Add a dependency to the backends, so that the backends will be built before this target
# to make sure the backends are always up-to-date when developing the target
# -----------------------------------------------------------------------------------------
function(uipc_target_add_backend_dependency target_name)
    add_dependencies(${target_name} uipc::backends)
endfunction()

function(uipc_target_add_include_files target_name)
    set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
    target_include_directories(${target_name} PUBLIC ${INCLUDE_DIR})
    file(GLOB_RECURSE INCLUDE_FILES "${INCLUDE_DIR}/*.h" "${INCLUDE_DIR}/*.inl")
    target_sources(${target_name} PRIVATE ${INCLUDE_FILES})

    # setup source group for the IDE
    source_group(TREE "${INCLUDE_DIR}" PREFIX "include" FILES ${INCLUDE_FILES})
endfunction()

function(uipc_init_submodule target)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${target}")
        uipc_error("Can not find submodule ${target} in ${CMAKE_CURRENT_SOURCE_DIR}, why?")
    endif()
    if (NOT UIPC_DEV_MODE)
        # NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${target}/.git"
        find_package(Git QUIET)
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init ${target}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            uipc_error("git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
        uipc_info("Submodule ${target} is initialized")
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Require a python module, if not found, try to install it with pip
# -----------------------------------------------------------------------------------------
function(uipc_require_python_module python_dir module_name)
execute_process(COMMAND ${python_dir}
    "-c" "import ${module_name}"
    RESULT_VARIABLE CMD_RESULT
    OUTPUT_QUIET
)
if (NOT CMD_RESULT EQUAL 0)
    uipc_info("${module_name} not found, try installing ${module_name}...")
    execute_process(COMMAND ${python_dir} "-m" "pip" "install" "${module_name}"
    RESULT_VARIABLE INSTALL_RESULT)
    if (NOT INSTALL_RESULT EQUAL 0)
        uipc_error("Python [${python_dir}] failed to install [${module_name}], please install it manually.")
    else()
        uipc_info("${module_name} installed successfully with [${python_dir}].")
    endif()
endif()
endfunction()

