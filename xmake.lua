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

rule("sail.cuda")
    on_load(function(target)
        local cuda_path = os.getenv("CUDA_PATH")
        if cuda_path then
            target:add("sysincludedirs", path.join(cuda_path, "include"), {public=true})
            target:add("linkdirs", path.join(cuda_path, "lib/x64/"), {public=true})
            target:add("links", "nvrtc", "cudart", "cuda", {public=true})
        else
            target:set("enabled", false)
            return
        end
        if is_plat("windows") then
            target:add("defines", "NOMINMAX", "UNICODE")
            target:add("syslinks", "Cfgmgr32", "Advapi32")
        end
    end)

    after_build(function(target)
        local cuda_path = os.getenv("CUDA_PATH")
        if cuda_path then
            shared_files = os.match(path.join(cuda_path, "bin/*.dll"))
            -- if shared files not exists(targetdir()), copy them
            local shared_file_names = {}
            for _, shared_file in ipairs(shared_files) do
                table.insert(shared_file_names, path.filename(shared_file))
            end
            for _, shared_file_name in ipairs(shared_file_names) do
                if not os.isfile(path.join(target:targetdir(), shared_file_name)) then
                    os.cp(path.join(cuda_path, "bin", shared_file_name), target:targetdir())
                    print("copy " .. shared_file_name .. " to " .. target:targetdir())
                end
            end
            -- os.cp(path.join(cuda_path, "lib/x64/cudadevrt.lib"), target:targetdir())
        end
    end)
rule_end()

target("muda")
    set_kind("headeronly")
    add_includedirs("external/muda/src/", {public = true})
    add_cugencodes("compute_75", {public = true})
    add_links("nvrtc", "cudart", "cuda", "cublas", "cusparse", "cusolver", {public = true})
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
add_requires("fmt", {version = "10.1.1"})
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