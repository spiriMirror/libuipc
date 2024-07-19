file(GLOB SOURCES "*.cpp" "*.cu" "*.h" "sym/*.inl" "details/*.inl")
target_sources(cuda PRIVATE ${SOURCES})
