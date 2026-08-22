file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp")

uipc_add_benchmark(basic)

target_sources(basic PRIVATE ${SOURCES})
uipc_target_set_relative_source_file(basic)