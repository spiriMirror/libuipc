file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc PRIVATE ${SOURCES})

add_subdirectory(utils)