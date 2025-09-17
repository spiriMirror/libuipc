add_requires(
    "eigen", "nlohmann_json", "cppitertools", "magic_enum", "tinygltf", "dylib <3", "cpptrace",
    -- Use non-header-only spdlog and fmt
    "spdlog[shared,header_only=n,fmt_external=y]"
)

if is_plat("windows") then
    -- https://forums.developer.nvidia.com/t/utf-8-option-for-the-host-function-in-cuda-msvc/312739
    add_requireconfs("spdlog.fmt", {override = true, configs = {unicode = false}})
else
    add_requireconfs("spdlog", "spdlog.fmt", {override = true, system = false})
end

add_defines("LUISA_USE_SYSTEM_STL", "_DISABLE_EXTENDED_ALIGNED_STORAGE", {public = true})

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
