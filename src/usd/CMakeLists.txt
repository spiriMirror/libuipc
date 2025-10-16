find_package(pxr REQUIRED CONFIG)
# print pxr version
message(STATUS "Found USD ${pxr_VERSION}")

add_library(uipc_usd SHARED)
add_library(uipc::usd ALIAS uipc_usd)

target_include_directories(uipc_usd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} 
    ${PROJECT_SOURCE_DIR}/include/uipc/usd/uipcPhysics)

uipc_target_add_include_files(uipc_usd)

target_link_libraries(uipc_usd PUBLIC 
    uipc::core
    uipc::geometry)

target_link_libraries(uipc_usd PUBLIC
   usd
   usdGeom
   usdPhysics
   ar gf js tf
)

target_compile_definitions(uipc_usd PRIVATE UIPC_USD_EXPORT_DLL=1) # export dll
target_compile_definitions(uipc_usd PRIVATE UIPCPHYSICS_EXPORTS=1) # export dll for usdPhysics

file(GLOB_RECURSE SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_usd PRIVATE ${SOURCES})

uipc_target_set_output_directory(uipc_usd)
uipc_target_set_rpath(uipc_usd)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})
