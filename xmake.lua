set_xmakever("2.9.2")
add_rules("mode.release", "mode.debug")
engine_version = "0.1.0"
set_languages("c++20")
add_undefines("min","max")

function version_major()
    return "0"
end

function version_minor()
    return "9"
end

function version_patch()
    return "0"
end

target("muda")
    set_kind("headeronly")
    add_includedirs("external/muda/src/", {public = true})
    add_cuflags("--extended-lambda", {public = true}) -- must be set for muda
    add_cuflags("--expt-relaxed-constexpr", {public = true}) -- must be set for muda
    add_cuflags("-rdc=true", {public = true})
target_end()

target("tetgen")
    set_kind("static")
    add_files("external/tetgen/tetgen/*.cxx")
    add_includedirs("external/tetgen/", {public = true})
target_end()

add_requires("boost[core]")
add_requires("eigen", {version="3.4.0"})
add_requires("catch2", {version="3.5.3"})
add_requires("spdlog", {version="1.12.0"})
add_requires("fmt[headeronly]", {version = "10.1.1"})
add_requires("cppitertools")
add_requires("tinygltf")
add_requires("dylib", {version = "2.2.1"})
add_requires("benchmark")
add_requires("nlohmann_json", {version = "3.11.2"})
add_requires("libigl")
add_requires("imgui")
add_requires("bgfx")
add_requires("glfw")
add_requires("magic_enum")
-- global include
add_includedirs("include")
add_includedirs("src")
includes("src")