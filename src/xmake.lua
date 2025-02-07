includes(
    "backends",
    "core",
    "geometry"
)

if has_config("pybind") then
    includes("pybind")
end

target("constitution")
    add_rules("component")
    add_files("constitution/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/constitution/*.h"))
    add_deps("geometry")

target("io")
    add_rules("component")
    add_files("io/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/io/*.h"))
    add_deps("geometry")

target("sanity_check")
    add_rules("component")
    add_files("sanity_check/*.cpp")
    add_includedirs("sanity_check")
    add_headerfiles("sanity_check/*.h", "sanity_check/details/*.inl")
    add_deps("geometry", "io")
