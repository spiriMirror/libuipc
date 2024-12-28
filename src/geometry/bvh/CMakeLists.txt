file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_geometry PRIVATE ${SOURCES})
