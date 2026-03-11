add_requires("muda 09f8a0beca898b5325c7b0c1e4cf67ea4781f3b9",{system = false,configs = {with_check = true}})

target("cuda")
    add_rules("backend")
    if has_config("dev") then
        add_rules("clangd")
    end
    add_files("**.cpp", "**.cu")
    add_headerfiles("**.h", "**.inl")
    add_includedirs(os.scriptdir(), {public = true})
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_rules("cuda_warning")
    add_rules("cuda_no_host_compiler_check")    
    add_cuflags("--expt-relaxed-constexpr")
    add_cuflags("--extended-lambda")
    
    add_links(
        "cudart",
        "cublas",
        "cusparse",
        "cusolver"
    )

    add_deps("uipc_geometry")
    on_load(function(target)
        if target:is_plat('windows') then
            target:add('defines', '__NV_NO_HOST_COMPILER_CHECK', {public = true})
            target:add('cuflags', '-allow-unsupported-compiler', {public = true})
            -- Suppress MSVC C4819 for host compilation of CUDA sources.
            target:add('cuflags', '-Xcompiler=/wd4819', {public = true})
            target:set('toolchains', 'msvc')
        end
        target:set('toolchains', 'cuda')
    end)
    add_packages("muda")

package("muda")
    set_kind("library", {headeronly = true})
    set_homepage("https://mugdxy.github.io/muda-doc")
    set_description("μ-Cuda, COVER THE LAST MILE OF CUDA. With features: intellisense-friendly, structured launch, automatic cuda graph generation and updating.")
    set_license("Apache-2.0")

    add_urls("https://github.com/MuGdxy/muda.git")

    set_policy("package.install_locally", true)

    add_configs("with_check", {description = "Enable muda check", default = true})
    add_configs("with_compute_graph", {description = "Enable muda compute graph", default = false})

    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr","-rdc=true",{public = true})


    on_install(function (package)
        local check_val = package:config("with_check") and "1" or "0"
        package:add('defines', 'MUDA_CHECK_ON=' .. check_val, {public = true})
        local compute_graph_val = package:config("with_compute_graph") and "1" or "0"
        package:add('defines', 'MUDA_COMPUTE_GRAPH_ON=' .. compute_graph_val, {public = true})
        
        os.cp("src/muda", package:installdir("include"))
    end)
