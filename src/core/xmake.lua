target("uipc_core")
    set_kind("shared")
    add_defines("UIPC_CORE_EXPORT_DLL")
    add_defines("UIPC_RUNTIME_CHECK")
    add_defines("UIPC_PROJECT_DIR=\"$(projectdir)\"", { public = true })
    add_defines("UIPC_VERSION_MAJOR=" .. version_major(), { public = true })
    add_defines("UIPC_VERSION_MINOR=" .. version_minor(), { public = true })
    add_defines("UIPC_VERSION_PATCH=" .. version_patch(), { public = true })
    
    add_packages(
        "boost",
        "eigen",
        "nlohmann_json",
        "cppitertools",
        "fmt",
        "spdlog",
        "magic_enum",
        "dylib",
        "tinygltf",
        "boost", { public = true }
    )
    add_files("**.cpp")
    add_defines("_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR", {public=true})
target_end()