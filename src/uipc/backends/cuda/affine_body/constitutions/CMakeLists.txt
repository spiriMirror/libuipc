file(GLOB SOURCES "*.cpp" "*.cu" "*.h" "sym/*.inl")
target_sources(cuda PRIVATE ${SOURCES})
