include(uipc_utils)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(libigl REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(cppitertools CONFIG REQUIRED)
find_path(DYLIB_INCLUDE_DIRS "dylib.hpp")

add_library(uipc SHARED)
add_library(uipc::uipc ALIAS uipc)
target_compile_definitions(uipc PRIVATE UIPC_CORE_EXPORT_DLL=1) # export dll

target_compile_features(uipc PUBLIC cxx_std_20)
target_include_directories(uipc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_include_directories(uipc PRIVATE "${DYLIB_INCLUDE_DIRS}")

target_link_libraries(uipc PUBLIC 
    Eigen3::Eigen 
    spdlog::spdlog 
    igl::igl_core
    nlohmann_json::nlohmann_json
    rapidcsv
    fmt::fmt
    cppitertools::cppitertools
)

# add the files in the subdirectories to the target uipc
# so ensure that the uipc target is created before adding the subdirectories
add_subdirectory(geometry)
add_subdirectory(common)
add_subdirectory(world)
add_subdirectory(backend)
add_subdirectory(builtin)
add_subdirectory(constitutions)
add_subdirectory(engine)
# add the source files in the current directory to the target uipc
file(GLOB SOURCE "*.h" "*.cpp")
target_sources(uipc PRIVATE ${SOURCE})

# Group files in IDE
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})

# add the backends to the target uipc
add_subdirectory(backends)

# if debug build or release build with debug info, enable uipc runtime check
target_compile_definitions(uipc PUBLIC 
    $<$<CONFIG:Debug>:UIPC_RUNTIME_CHECK=1>
    $<$<CONFIG:RelWithDebInfo>:UIPC_RUNTIME_CHECK=1>)
target_compile_definitions(uipc PUBLIC 
    $<$<CONFIG:Release>:UIPC_RUNTIME_CHECK=0>)

uipc_set_output_directory(uipc)