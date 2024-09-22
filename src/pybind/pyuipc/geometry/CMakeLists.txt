file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(pyuipc PRIVATE ${SOURCES})

add_subdirectory(utils)
