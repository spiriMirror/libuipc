file(GLOB SOURCES "*.cpp")

uipc_add_benchmark(basic)

target_sources(basic PRIVATE ${SOURCES})
uipc_target_set_relative_source_file(basic)