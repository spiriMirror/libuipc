set_xmakever("2.9.2")

add_rules("mode.release", "mode.debug", "mode.releasedbg")
engine_version = "0.1.0"
set_languages("c++20")
add_undefines("min","max")
add_cxxflags("/NODEFAULTLIB:libcmt")

if is_mode("debug") then
    set_targetdir("bin/debug")
    set_runtimes("MDd")
elseif is_mode("releasedbg") then
    set_targetdir("bin/releasedbg")
    set_runtimes("MD")
else
    set_targetdir("bin/release")
    set_runtimes("MD")
end

option("uipc_enable_pybind")
    set_default(true)
    set_showmenu(true)
option_end()

function version_major()
    return "0"
end

function version_minor()
    return "9"
end

function version_patch()
    return "0"
end

rule("uipc.predef")
    on_load(function(target)
        target:add("defines", "_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR", {public=true})
        -- see https://developercommunity.visualstudio.com/t/All-my-std::unique_lock-crashed-after-th/10665376?space=41&sort=newest&viewtype=all
        -- https://www.soinside.com/question/VTab6FvWLARJWzigEhyBeC
        -- https://github.com/microsoft/STL/wiki/Changelog mutex constexpr constructor
    end)
rule_end()

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

add_requires("boost", { configs = { core = true }})
add_requires("eigen", {version="3.4.0"})
add_requires("catch2", {version="3.5.3"})
-- add_requires("fmt[headeronly]", {version = "10.1.1"})
add_requires("fmt", {version = "10.2.1"})
add_requires("spdlog", {version = "1.9.2"})
add_requires("cppitertools")
add_requires("tinygltf")
add_requires("dylib", {version = "2.2.1"})
add_requires("benchmark")
add_requires("nlohmann_json", {version = "3.11.2"})
add_requires("libigl")
add_requires("imgui")
add_requires("glfw")
add_requires("magic_enum")

if has_config("uipc_enable_pybind") then
    add_requires("python[3.11.9]")
    add_requires("pybind11")
end
-- global include
add_includedirs("include")
add_includedirs("src")
includes("src")
includes("apps")

