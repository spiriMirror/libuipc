
target("uipc_backend_none")
    set_kind("shared")
    add_defines("UIPC_BACKEND_EXPORT_DLL")
    add_defines("UIPC_BACKEND_DIR=\"$(targetdir)\"")
    add_defines("UIPC_BACKEND_NAME=\"none\"")
    add_deps("uipc_core", "uipc_geometry", { public = true })

    add_files("**.cpp")
    add_files("../common/**.cpp")
    -- add inl
    add_includedirs(".")
    add_includedirs("../common")
    -- add_defines("FMT_EXPORT")
target_end()
