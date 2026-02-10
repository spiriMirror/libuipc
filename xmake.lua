set_xmakever("3.0.5")

option("gui", {default = false})
option("pybind", {default = false, description = "Build pyuipc"})
option("torch", {default = false, description = "Build pytorch extension"})
option("examples", {default = true})
option("tests", {default = true})
option("benchmarks", {default = false})
option("dev", {default = true, description = "Enable developer mode"})
option("github_actions", {default = false})

option("backend_cuda", {default = true, description = "Build with CUDA backend"})

option("python_version", {default = "3.11.x", description = "Specify python version"})
option("python_system", {default = false, description = "Use system python"})


includes("src", "apps", "xmake/*.lua")

add_rules("mode.release", "mode.debug", "mode.releasedbg", "uipc.basic")

set_languages("c++20")

if is_plat("linux") then
    add_rpathdirs("$ORIGIN")
end

set_version("0.9.0")

if has_config("dev") then
    set_policy("compatibility.version", "3.0")
    set_policy("build.ccache", true)

    if is_plat("windows") then
        set_runtimes("MD")
    end
end
