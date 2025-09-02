set_xmakever("2.9.8")

option("gui", {default = false})
option("pybind", {default = true, description = "Build pyuipc"})
option("torch", {default = false, description = "Build pytorch extension"})
option("examples", {default = true})
option("tests", {default = true})
option("benchmarks", {default = false})
option("dev", {default = true, description = "Enable developer mode"})
option("github_actions", {default = false})

option("backend", {default = "cuda", description = "Build with CUDA backend"})

option("python_version", {default = "3.11.x", description = "Specify python version"})
option("python_system", {default = false, description = "Use system python"})

includes("src", "xmake/*.lua")

add_rules("mode.release", "mode.debug", "mode.releasedbg")
add_rules("plugin.compile_commands.autoupdate", {outputdir = ".vscode"})

set_languages("c++20")

if is_plat("windows") then
    add_cxflags("/FC", {force = true})
    add_cxflags("/wd4068", {force = true})
    add_cxxflags("/wd4068", {force = true})
else
    local project_dir = os.projectdir()
    add_cxflags("-fmacro-prefix-map=" .. project_dir .. "=.", {force = true})
end
add_cxflags("/wd4068", {force = true})

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

includes("apps")