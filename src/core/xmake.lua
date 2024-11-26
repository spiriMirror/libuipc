target("uipc_core")
    set_kind("shared")
    add_defines("UIPC_CORE_EXPORT_DLL")
    add_defines("UIPC_RUNTIME_CHECK")
    add_defines("UIPC_PROJECT_DIR=\"$(projectdir)\"")
    add_defines("UIPC_VERSION_MAJOR=" .. version_major())
    add_defines("UIPC_VERSION_MINOR=" .. version_minor())
    add_defines("UIPC_VERSION_PATCH=" .. version_patch())
    add_packages(
        "eigen",
        "spdlog",
        "nlohmann_json",
        "cppitertools",
        "fmt",
        "magic_enum",
        "dylib",
        "tinygltf",
        "boost", { public = true }
    )
    add_files("**.cpp")
target_end()