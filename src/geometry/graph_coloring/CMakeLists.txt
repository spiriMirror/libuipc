file(GLOB SOURCES "*.cpp" "*.hpp" "details/*.inl")
target_sources(uipc_geometry PRIVATE ${SOURCES})