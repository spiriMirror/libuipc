add_requires(
    "eigen", "nlohmann_json", "cppitertools", "magic_enum", "tinygltf", "dylib",
    "boost[header_only=y]",
    -- Use non-header-only spdlog and fmt
    "spdlog[header_only=n,fmt_external=y]"
)

-- https://stackoverflow.com/questions/78935510/no-member-named-join-in-namespace-fmt
add_requireconfs("spdlog.fmt", {override = true, version = "<11"})

target("core")
    add_rules("component")
    add_files("**.cpp")
    add_headerfiles(
        path.join(os.projectdir(), "include/uipc/core/**.h"),
        path.join(os.projectdir(), "include/uipc/core/**.inl")
    )

    add_defines("UIPC_RUNTIME_CHECK")

    if is_plat("linux") then
        add_syslinks("dl")
    end

    add_packages(
        "eigen", "nlohmann_json", "cppitertools", "magic_enum", "tinygltf", "dylib",
        "boost", "spdlog",
        {public = true}
    )
