target("uipc_geometry")
    set_kind("shared")
    add_defines("UIPC_GEOMETRY_EXPORT_DLL")
    add_deps("uipc_core", "tetgen", { public = true })
    add_packages("libigl", { public = true })
    add_files("**.cpp")
target_end()