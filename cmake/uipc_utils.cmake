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
    message(STATUS "    * UIPC_BUILD_PYTHON_WHEEL: ${UIPC_BUILD_PYTHON_WHEEL}")
    message(STATUS "    * UIPC_USING_LOCAL_VCPKG: ${UIPC_USING_LOCAL_VCPKG}")
    message(STATUS "    * UIPC_BUILD_EXAMPLES: ${UIPC_BUILD_EXAMPLES}")
    message(STATUS "    * UIPC_BUILD_TESTS: ${UIPC_BUILD_TESTS}")
    message(STATUS "    * UIPC_BUILD_BENCHMARKS: ${UIPC_BUILD_BENCHMARKS}")

    message(STATUS "    * UIPC_WITH_USD_SUPPORT: ${UIPC_WITH_USD_SUPPORT}")
    message(STATUS "    * UIPC_USD_INSTALL_DIR: ${UIPC_USD_INSTALL_DIR}")

    message(STATUS "    * UIPC_WITH_VDB_SUPPORT: ${UIPC_WITH_VDB_SUPPORT}")
    message(STATUS "    * UIPC_PYTHON_EXECUTABLE_PATH: ${UIPC_PYTHON_EXECUTABLE_PATH}")

    message(STATUS "Backend Options:")
    message(STATUS "    * UIPC_WITH_CUDA_BACKEND: ${UIPC_WITH_CUDA_BACKEND}")
    message(STATUS "    * UIPC_CUDA_ARCHITECTURES: ${UIPC_CUDA_ARCHITECTURES}")
    
    message(STATUS "Details:")
    message(STATUS "SKBUILD: ${SKBUILD}")
    message(STATUS "UIPC_INSTALL_DIR: ${UIPC_INSTALL_DIR}")
    message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
    message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
    message(STATUS "CMAKE_BUILD_PARALLEL_LEVEL: $ENV{CMAKE_BUILD_PARALLEL_LEVEL}")
    message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
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
    else()
        # make the UIPC_PYTHON_EXECUTABLE_PATH to a cmake path
        file(TO_CMAKE_PATH "${UIPC_PYTHON_EXECUTABLE_PATH}" UIPC_PYTHON_EXECUTABLE_PATH)
        set(Python_EXECUTABLE "${UIPC_PYTHON_EXECUTABLE_PATH}" CACHE STRING "Python executable path" FORCE)
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
    uipc_find_python_executable_path()
    # call python script to generate vcpkg.json, pass the CMAKE_BINARY_DIR as argument
    execute_process(
        COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_vcpkg_json.py"
        ${VCPKG_MANIFEST_DIR} # pass the CMAKE_CURRENT_BINARY_DIR as vcpkg.json output directory
        "--build_gui=${UIPC_BUILD_GUI}" # pass the UIPC_BUILD_GUI as argument
        "--dev_mode=${UIPC_DEV_MODE}" # pass the UIPC_DEV_MODE as argument
        "--with_usd_support=${UIPC_WITH_USD_SUPPORT}" # pass the UIPC_WITH_USD_SUPPORT as argument
        "--with_vdb_support=${UIPC_WITH_VDB_SUPPORT}" # pass the UIPC_WITH_VDB_SUPPORT as argument
        "--with_cuda_backend=${UIPC_WITH_CUDA_BACKEND}" # pass the UIPC_WITH_CUDA_BACKEND as argument
        OUTPUT_VARIABLE VCPKG_JSON_GENERATE_OUTPUT
        RESULT_VARIABLE VCPKG_JSON_GENERATE_RESULT
        ECHO_OUTPUT_VARIABLE # also print output to console
    )

    # check if the script ran successfully (exit code 0)
    if(NOT VCPKG_JSON_GENERATE_RESULT EQUAL 0)
        uipc_error("gen_vcpkg_json.py failed with exit code ${VCPKG_JSON_GENERATE_RESULT}")
    endif()

    # parse the output to determine if vcpkg install is needed
    # the script outputs "VCPKG_MANIFEST_CHANGED=1" or "VCPKG_MANIFEST_CHANGED=0"
    if(VCPKG_JSON_GENERATE_OUTPUT MATCHES "VCPKG_MANIFEST_CHANGED=1")
        set(VCPKG_MANIFEST_INSTALL ON CACHE BOOL "" FORCE)
    else()
        set(VCPKG_MANIFEST_INSTALL OFF CACHE BOOL "" FORCE)
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

# -----------------------------------------------------------------------------------------
# Set UIPC_RELATIVE_SOURCE_FILE for each source of a target
# -----------------------------------------------------------------------------------------
function(uipc_target_set_relative_source_file target_name)
    # Ref: https://github.com/spiriMirror/libuipc/issues/288
    get_target_property(TARGET_SOURCES ${target_name} SOURCES)
    if(NOT TARGET_SOURCES)
        return()
    endif()
    foreach(source IN LISTS TARGET_SOURCES)
        if(source MATCHES "\\$<")
            continue()
        endif()
        if(IS_ABSOLUTE "${source}")
            set(ABS_SOURCE "${source}")
        else()
            cmake_path(ABSOLUTE_PATH source
                       BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                       OUTPUT_VARIABLE ABS_SOURCE)
        endif()
        cmake_path(RELATIVE_PATH ABS_SOURCE
                   BASE_DIRECTORY "${PROJECT_SOURCE_DIR}"
                   OUTPUT_VARIABLE RELATIVE_PATH)
        set_property(SOURCE "${source}" APPEND PROPERTY
                     COMPILE_DEFINITIONS "UIPC_RELATIVE_SOURCE_FILE=R\"(${RELATIVE_PATH})\"")
    endforeach()
endfunction()


# -----------------------------------------------------------------------------------------
# Initialize a submodule
# -----------------------------------------------------------------------------------------
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
# Require pip module, if not found, try to install it
# -----------------------------------------------------------------------------------------
function(uipc_require_pip_ensure python_dir)
    execute_process(COMMAND ${python_dir}
        "-c" "import pip"
        RESULT_VARIABLE CMD_RESULT
        OUTPUT_QUIET
    )

    if (NOT CMD_RESULT EQUAL 0)
        uipc_info("pip not available, trying ensurepip...")
        execute_process(COMMAND ${python_dir} "-m" "ensurepip" "--upgrade"
            RESULT_VARIABLE ENSUREPIP_RESULT)
        if (NOT ENSUREPIP_RESULT EQUAL 0)
            uipc_error("Python [${python_dir}] failed to bootstrap pip. Please install pip manually.")
        endif()
    endif()
endfunction()




# -----------------------------------------------------------------------------------------
# Require a python module, if not found, try to install it with pip
# -----------------------------------------------------------------------------------------
function(uipc_require_python_module python_dir module_name)
    uipc_require_pip_ensure(${python_dir})

    file(TO_CMAKE_PATH "${python_dir}" python_dir)
    uipc_info("Check python module [${module_name}] with [${python_dir}]")

    # check if the module is installed
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
            uipc_info("[${module_name}] installed successfully with [${python_dir}].")
        endif()
    else()
        uipc_info("[${module_name}] found with [${python_dir}].")
    endif()
endfunction()


# -----------------------------------------------------------------------------------------
# Install headers
# -----------------------------------------------------------------------------------------
function(uipc_install_headers)
    if(NOT UIPC_BUILD_PYTHON_WHEEL)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION ${UIPC_INSTALL_DIR}/include
        FILES_MATCHING
        PATTERN "*.*")
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Install a target
# -----------------------------------------------------------------------------------------
function(uipc_install_target target_name)
    set(CONFIG_DIR "$<CONFIG>")
    if(NOT WIN32)
        if("${CMAKE_BUILD_TYPE}" STREQUAL "")
            set(CONFIG_DIR "Release")
        endif()
    endif()

    if(NOT UIPC_BUILD_PYTHON_WHEEL)
        # C++ package install logic
        install(TARGETS ${target_name}
            RUNTIME DESTINATION "${UIPC_INSTALL_DIR}/${CONFIG_DIR}/bin"
            LIBRARY DESTINATION "${UIPC_INSTALL_DIR}/${CONFIG_DIR}/bin"
            ARCHIVE DESTINATION "${UIPC_INSTALL_DIR}/${CONFIG_DIR}/lib")
    else()
        install(TARGETS ${target_name}
            RUNTIME DESTINATION "${UIPC_INSTALL_DIR}"
            LIBRARY DESTINATION "${UIPC_INSTALL_DIR}"
            ARCHIVE DESTINATION EXCLUDE_FROM_ALL)
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Install all vcpkg runtime dependencies (DLLs/shared libraries) to UIPC_INSTALL_DIR
# This installs all DLLs from vcpkg_installed/bin and lib directories
# -----------------------------------------------------------------------------------------
function(uipc_install_vcpkg_runtime)
    if(NOT DEFINED VCPKG_INSTALLED_DIR OR NOT DEFINED VCPKG_TARGET_TRIPLET)
        uipc_error("VCPKG_INSTALLED_DIR or VCPKG_TARGET_TRIPLET is not defined")
    endif()
    
    set(CONFIG_DIR "$<CONFIG>")
    if(NOT WIN32)
        if("${CMAKE_BUILD_TYPE}" STREQUAL "")
            set(CONFIG_DIR "Release")
        else()
            set(CONFIG_DIR "${CMAKE_BUILD_TYPE}")
        endif()
    endif()
    
    set(INSTALL_DESTINATION "")
    if(NOT UIPC_BUILD_PYTHON_WHEEL)
        set(INSTALL_DESTINATION "${UIPC_INSTALL_DIR}/${CONFIG_DIR}")
    else()
        set(INSTALL_DESTINATION "${UIPC_INSTALL_DIR}")
    endif()

    # Install runtime dependencies from vcpkg_installed
    # Check all possible directories (bin, lib, debug/bin, debug/lib) on all platforms
    if(EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
        install(DIRECTORY "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin/"
            DESTINATION "${INSTALL_DESTINATION}"
            FILES_MATCHING PATTERN "*.*"
        )
    endif()

    if(NOT UIPC_BUILD_PYTHON_WHEEL)
        # C++ package install logic
        if(EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib")
            install(DIRECTORY "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib/"
                DESTINATION "${INSTALL_DESTINATION}/lib"
                FILES_MATCHING PATTERN "*.*"
            )
        endif()
        
        if(EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/lib")
            install(DIRECTORY "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/lib/"
                DESTINATION "${INSTALL_DESTINATION}/debug/lib"
                FILES_MATCHING PATTERN "*.*"
            )
        endif()

        if(EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin")
            install(DIRECTORY "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin/"
                DESTINATION "${INSTALL_DESTINATION}/debug/bin"
                FILES_MATCHING PATTERN "*.*"
            )
        endif()
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Set RPATH for the target
# -----------------------------------------------------------------------------------------
function(uipc_target_set_rpath target_name)
    if(APPLE)
        # macOS: @executable_path
        set_target_properties(${target_name} PROPERTIES INSTALL_RPATH "@executable_path")
        set_target_properties(${target_name} PROPERTIES BUILD_RPATH "@executable_path")
    elseif(UNIX)
        # Linux and other Unix systems use $ORIGIN
        set_target_properties(${target_name} PROPERTIES INSTALL_RPATH "$ORIGIN")
        set_target_properties(${target_name} PROPERTIES BUILD_RPATH "$ORIGIN")
    endif()
endfunction()


# -----------------------------------------------------------------------------------------
# Add Vcpkg Shared Library PATH to the executable target
# -----------------------------------------------------------------------------------------
function(uipc_executable_add_vcpkg_rpath target_name)
    set(BASIC_RPATH "")
    if(APPLE)
        set(BASIC_RPATH "@executable_path")
    elseif(UNIX)
        set(BASIC_RPATH "$ORIGIN")
    endif()

    set(VCPKG_RPATH "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
    set(RELEASE_RPATH "${BASIC_RPATH};${VCPKG_RPATH}/lib;${VCPKG_RPATH}/bin")
    set(DEBUG_RPATH "${BASIC_RPATH};${VCPKG_RPATH}/debug/lib;${VCPKG_RPATH}/debug/bin")

    if(NOT WIN32)
        set(INSTALL_RPATH_GENEX "$<IF:$<CONFIG:Debug>,${DEBUG_RPATH},${RELEASE_RPATH}>")
        set(BUILD_RPATH_GENEX "$<IF:$<CONFIG:Debug>,${DEBUG_RPATH},${RELEASE_RPATH}>")
        set_target_properties(${target_name} PROPERTIES
                INSTALL_RPATH "${INSTALL_RPATH_GENEX}"
                BUILD_RPATH "${BUILD_RPATH_GENEX}"
        )
    endif()
    # For Windows, RPATH is not a standard mechanism. The most reliable way to ensure
    # DLLs are found is to copy them to the executable's directory, which can be
    # done with a post-build command. This function will not set RPATH for Windows.
endfunction()