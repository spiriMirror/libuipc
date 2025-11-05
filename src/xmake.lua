includes(
    "backends",
    "core",
    "geometry"
)

if has_config("pybind") then
    includes("pybind")
end

if has_config("grpc") then
    includes("rpc")
end

add_requires("urdfdom")

target("uipc_constitution")
    add_rules("component")
    add_files("constitution/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/constitution/*.h"))
    add_deps("uipc_geometry")

target("uipc_io")
    add_rules("component")
    add_files("io/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/io/*.h"))
    add_deps("uipc_geometry")
    add_packages("urdfdom")
    -- undef far near on windows to avoid conflicts
    if is_plat("windows") then
        add_defines("_UIPC_IO_UNDEF_FAR_NEAR")
    end

target("uipc_sanity_check")
    add_rules("component")
    add_files("sanity_check/*.cpp")
    add_includedirs("sanity_check")
    add_headerfiles("sanity_check/*.h", "sanity_check/details/*.inl")
    add_deps("uipc_geometry", "uipc_io")

rule("uipc_deps")
    on_load(function (target)
        target:add("deps","uipc_core", "uipc_geometry", "uipc_io","uipc_constitution","uipc_sanity_check")
    end)
