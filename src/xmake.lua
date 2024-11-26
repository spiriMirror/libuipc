includes("core")
includes("geometry")
includes("constitution")
includes("backends")
includes("io")
includes("sanity_check")

target("uipc_uipc")
    set_kind("phony")
    add_deps(
        "uipc_core", 
        "uipc_geometry", 
        "uipc_constitution", 
        "uipc_io", 
        "uipc_sanity_check", { public = true })
target_end()