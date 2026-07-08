add_requires(
    "eigen", "nlohmann_json", "cppitertools", "magic_enum", "tinygltf", "dylib <3", "cpptrace",
    -- Use non-header-only spdlog and fmt
    "spdlog[shared,header_only=n,fmt_external=y]"
)

-- Pin fmt to 12.1.0: 12.2.0's long double hexfloat path fails under nvcc (fallback uint128 lacks operator~)
if is_plat("windows") then
    -- https://forums.developer.nvidia.com/t/utf-8-option-for-the-host-function-in-cuda-msvc/312739
    add_requireconfs("spdlog.fmt", {override = true, version = "12.1.0", configs = {unicode = false}})
else
    add_requireconfs("spdlog", {override = true, system = false})
    add_requireconfs("spdlog.fmt", {override = true, system = false, version = "12.1.0"})
end

target("uipc_core")
    add_rules("component")
    add_files("**.cpp")
    add_headerfiles(
        path.join(os.projectdir(), "include/uipc/core/**.h"),
        path.join(os.projectdir(), "include/uipc/core/**.inl")
    )

    add_defines("UIPC_RUNTIME_CHECK=1", {public = true})

    if is_plat("linux") then
        add_syslinks("dl")
    end

    add_packages(
        "eigen", "nlohmann_json", "cppitertools", "magic_enum", "tinygltf", "dylib",
        "cpptrace", "spdlog",
        {public = true}
    )
