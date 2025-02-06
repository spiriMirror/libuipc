set_xmakever("2.9.8")

option("gui", {default = false})
option("pybind", {default = false, description = "Build pyuipc"})
option("torch", {default = false, description = "Build pytorch extension"})
option("examples", {default = false})
option("tests", {default = false})
option("benchmarks", {default = false})
option("dev", {default = true, description = "Enable developer mode"})

option("backend", {default = "cuda", description = "Build with CUDA backend"})

includes("src", "xmake/rules.lua")

add_rules("mode.release", "mode.debug", "mode.releasedbg")

set_languages("c++20")

set_version("0.9.0")

if has_config("dev") then
    set_policy("compatibility.version", "3.0")
    if is_plat("windows") then
        set_runtimes("MD")
    end
end
