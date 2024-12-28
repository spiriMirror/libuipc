target("uipc_sanity_check")
    set_kind("shared")
    add_includedirs(".", { public = true })
    add_deps(
        "uipc_core", 
        "uipc_geometry", 
        "uipc_io",
    { public = true })
    add_files("*.cpp")
target_end()