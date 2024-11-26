includes("none")
includes("cuda")

target("uipc_backends")
    set_kind("phony")
    add_deps("uipc_backend_cuda")
target_end()