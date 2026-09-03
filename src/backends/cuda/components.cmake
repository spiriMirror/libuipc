function(uipc_add_cuda_component name)
    add_library(${name} OBJECT ${ARGN})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_include_directories(${name} PRIVATE
        "${PROJECT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/cuda_tool"
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
    if(CUDAToolkit_VERSION_MAJOR GREATER_EQUAL 13)
        target_include_directories(${name} PRIVATE ${CUDAToolkit_INCLUDE_DIRS}/cccl)
    endif()
    target_compile_definitions(${name} PRIVATE
        UIPC_BACKEND_EXPORT_DLL=1
        "-DUIPC_BACKEND_DIR=R\"(${UIPC_BACKENDS_SOURCE_DIR})\""
        "-DUIPC_BACKEND_NAME=R\"(cuda)\"")
    target_compile_options(${name} PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:--diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028>
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
        $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)
    target_link_libraries(${name} PRIVATE
        uipc::core
        uipc_geometry
        uipc_io)
    set_target_properties(${name} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_STANDARD_REQUIRED ON
        CUDA_STANDARD 20
        FOLDER "uipc-backends/cuda-components")
    uipc_set_target_cuda_architectures(${name} "${UIPC_CUDA_ARCHITECTURES}")
    uipc_target_set_relative_source_file(${name})
    target_sources(cuda PRIVATE $<TARGET_OBJECTS:${name}>)
endfunction()

file(GLOB UIPC_CUDA_RUNTIME_ROOT_SOURCES CONFIGURE_DEPENDS "*.cu" "*.cpp")
file(GLOB_RECURSE UIPC_CUDA_RUNTIME_DOMAIN_SOURCES CONFIGURE_DEPENDS
    "animator/*.cu" "animator/*.cpp"
    "diff_sim/*.cu" "diff_sim/*.cpp"
    "engine/*.cu" "engine/*.cpp"
    "external_force/*.cu" "external_force/*.cpp"
    "global_geometry/*.cu" "global_geometry/*.cpp"
    "implicit_geometry/*.cu" "implicit_geometry/*.cpp"
    "joint_dof_system/*.cu" "joint_dof_system/*.cpp"
    "line_search/*.cu" "line_search/*.cpp"
    "newton_tolerance/*.cu" "newton_tolerance/*.cpp"
    "pipeline/*.cu" "pipeline/*.cpp"
    "sanity_check/*.cu" "sanity_check/*.cpp"
    "time_integrator/*.cu" "time_integrator/*.cpp"
    "utils/*.cu" "utils/*.cpp")
set(UIPC_CUDA_RUNTIME_SOURCES
    ${UIPC_CUDA_RUNTIME_ROOT_SOURCES}
    ${UIPC_CUDA_RUNTIME_DOMAIN_SOURCES})

file(GLOB_RECURSE UIPC_CUDA_AFFINE_BODY_SOURCES CONFIGURE_DEPENDS
    "affine_body/*.cu" "affine_body/*.cpp")
file(GLOB_RECURSE UIPC_CUDA_COLLISION_SOURCES CONFIGURE_DEPENDS
    "collision_detection/*.cu" "collision_detection/*.cpp")
set(UIPC_CUDA_COLLISION_LEGACY_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/collision_detection/filters/info_stackless_bvh_v0_simplex_trajectory_filter.cu"
    "${CMAKE_CURRENT_SOURCE_DIR}/collision_detection/filters/lbvh_simplex_trajectory_filter.cu"
    "${CMAKE_CURRENT_SOURCE_DIR}/collision_detection/filters/stackless_bvh_simplex_trajectory_filter.cu")
list(REMOVE_ITEM UIPC_CUDA_COLLISION_SOURCES
    ${UIPC_CUDA_COLLISION_LEGACY_SOURCES})
file(GLOB_RECURSE UIPC_CUDA_CONTACT_SOURCES CONFIGURE_DEPENDS
    "active_set_system/*.cu" "active_set_system/*.cpp"
    "contact_system/*.cu" "contact_system/*.cpp"
    "distance_system/*.cu" "distance_system/*.cpp"
    "dytopo_effect_system/*.cu" "dytopo_effect_system/*.cpp"
    "inter_primitive_effect_system/*.cu" "inter_primitive_effect_system/*.cpp")
file(GLOB_RECURSE UIPC_CUDA_FEM_SOURCES CONFIGURE_DEPENDS
    "finite_element/*.cu" "finite_element/*.cpp")
file(GLOB_RECURSE UIPC_CUDA_LINEAR_SYSTEM_SOURCES CONFIGURE_DEPENDS
    "linear_system/*.cu" "linear_system/*.cpp")
file(GLOB_RECURSE UIPC_CUDA_COUPLING_SOURCES CONFIGURE_DEPENDS
    "coupling_system/*.cu" "coupling_system/*.cpp")

set(UIPC_CUDA_COMPONENT_SOURCES
    ${UIPC_CUDA_RUNTIME_SOURCES}
    ${UIPC_CUDA_AFFINE_BODY_SOURCES}
    ${UIPC_CUDA_COLLISION_SOURCES}
    ${UIPC_CUDA_COLLISION_LEGACY_SOURCES}
    ${UIPC_CUDA_CONTACT_SOURCES}
    ${UIPC_CUDA_FEM_SOURCES}
    ${UIPC_CUDA_LINEAR_SYSTEM_SOURCES}
    ${UIPC_CUDA_COUPLING_SOURCES})
set(UIPC_CUDA_UNIQUE_COMPONENT_SOURCES ${UIPC_CUDA_COMPONENT_SOURCES})
list(REMOVE_DUPLICATES UIPC_CUDA_UNIQUE_COMPONENT_SOURCES)
list(LENGTH UIPC_CUDA_COMPONENT_SOURCES UIPC_CUDA_COMPONENT_SOURCE_COUNT)
list(LENGTH UIPC_CUDA_UNIQUE_COMPONENT_SOURCES UIPC_CUDA_UNIQUE_SOURCE_COUNT)
if(NOT UIPC_CUDA_COMPONENT_SOURCE_COUNT EQUAL UIPC_CUDA_UNIQUE_SOURCE_COUNT)
    message(FATAL_ERROR "A CUDA source belongs to more than one internal component")
endif()

file(GLOB_RECURSE UIPC_CUDA_ALL_COMPILE_SOURCES CONFIGURE_DEPENDS "*.cu" "*.cpp")
foreach(source IN LISTS UIPC_CUDA_ALL_COMPILE_SOURCES)
    if(NOT source IN_LIST UIPC_CUDA_UNIQUE_COMPONENT_SOURCES)
        message(FATAL_ERROR "CUDA source has no internal component: ${source}")
    endif()
endforeach()
foreach(source IN LISTS UIPC_CUDA_UNIQUE_COMPONENT_SOURCES)
    if(NOT source IN_LIST UIPC_CUDA_ALL_COMPILE_SOURCES)
        message(FATAL_ERROR "CUDA component source is outside the backend inventory: ${source}")
    endif()
endforeach()

uipc_add_cuda_component(cuda_runtime_objects ${UIPC_CUDA_RUNTIME_SOURCES})
uipc_add_cuda_component(cuda_affine_body_objects ${UIPC_CUDA_AFFINE_BODY_SOURCES})
uipc_add_cuda_component(cuda_collision_objects ${UIPC_CUDA_COLLISION_SOURCES})
if(UIPC_WITH_CUDA_LEGACY_COLLISION)
    uipc_add_cuda_component(cuda_collision_legacy_objects
        ${UIPC_CUDA_COLLISION_LEGACY_SOURCES})
endif()
uipc_add_cuda_component(cuda_contact_objects ${UIPC_CUDA_CONTACT_SOURCES})
uipc_add_cuda_component(cuda_fem_objects ${UIPC_CUDA_FEM_SOURCES})
uipc_add_cuda_component(cuda_linear_system_objects ${UIPC_CUDA_LINEAR_SYSTEM_SOURCES})
uipc_add_cuda_component(cuda_coupling_objects ${UIPC_CUDA_COUPLING_SOURCES})
