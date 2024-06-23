add_subdirectory(visitors)
file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc PRIVATE ${SOURCES})
