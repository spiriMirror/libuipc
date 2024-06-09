file(GLOB SOURCES "*.cpp")

uipc_add_benchmark(basic)

target_sources(basic PRIVATE ${SOURCES})