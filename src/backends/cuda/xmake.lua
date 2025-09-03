add_requires("muda 5b177ad0f450183914300ee55e481bf01c673d5d")

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
    add_cuflags("/wd4819", {tools = "cl"})
    add_cuflags("--diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,550", {force = true})
    add_links(
        "cudart",
        "cublas",
        "cusparse",
        "cusolver"
    )

    add_deps("uipc_geometry")
    add_packages("muda")

package("muda")
    set_kind("library", {headeronly = true})
    set_homepage("https://mugdxy.github.io/muda-doc")
    set_description("μ-Cuda, COVER THE LAST MILE OF CUDA. With features: intellisense-friendly, structured launch, automatic cuda graph generation and updating.")
    set_license("Apache-2.0")

    add_urls("https://github.com/MuGdxy/muda.git")

    add_versions("2025.02.28", "ff8558b8842247787545353e7d370ae376f212c5")
    add_versions("2025.02.01", "008b6fdd48e6ffa7bcaf79943beb24f940d8da93")

    set_policy("package.install_locally", true)

    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr")

    on_install(function (package)
        os.cp("src/muda", package:installdir("include"))
    end)
