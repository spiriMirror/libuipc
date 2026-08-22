file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.hpp" "details/*.inl")
target_sources(uipc_geometry PRIVATE ${SOURCES})