file(GLOB SOURCES "*.cpp" "*.cu" "*.h" "details/*.inl")
target_sources(cuda PRIVATE ${SOURCES})

add_subdirectory(constitutions)
add_subdirectory(constraints)