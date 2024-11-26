target("uipc_backend_cuda")
    set_kind("shared")
    add_defines("UIPC_BACKEND_EXPORT_DLL")
    add_defines("UIPC_BACKEND_DIR=\"$(targetdir)\"")
    add_defines("UIPC_BACKEND_NAME=\"cuda\"")
    add_packages("magic_enum")
    add_deps("uipc_core", "uipc_geometry", { public = true })
    add_deps("muda", { public = true })

    add_cugencodes("compute_75", {public = true})
    add_links("nvrtc", "cudart", "cuda", "cublas", "cusparse", "cusolver", {public = true})
    
    add_files("**.cu", "**.cpp")
    add_files("../common/**.cpp")
    add_files("../none/**.cpp")
    -- add inl
    add_includedirs(".")
    add_includedirs("../common")
    add_includedirs("../none/")


target_end()

target("uipc_backends")
    set_kind("phony")
    add_deps("uipc_backend_cuda")
target_end()