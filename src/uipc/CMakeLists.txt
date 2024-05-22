find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(uipc STATIC)
target_compile_features(uipc PUBLIC cxx_std_20)

add_subdirectory(geometry)
add_subdirectory(common)

file(GLOB UIPC_SOURCE_FILES "*.h" "*.cpp")
target_sources(uipc PRIVATE ${UIPC_SOURCE_FILES})

target_include_directories(uipc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(uipc PUBLIC Eigen3::Eigen spdlog::spdlog)


# Group files in IDE
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})