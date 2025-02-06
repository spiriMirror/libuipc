add_requires("muda")

target("cuda")
    add_rules("backend")
    add_files("**.cpp", "**.cu")
    add_headerfiles("**.h", "**.inl")

    add_cugencodes("sm_75")
    add_cuflags("/wd4819", {tools = "cl"})

    add_links(
        "cublas",
        "cusparse",
        "cusolver"
    )

    add_deps("geometry")
    add_packages("muda")

package("muda")
    set_kind("library", {headeronly = true})
    set_homepage("https://mugdxy.github.io/muda-doc")
    set_description("μ-Cuda, COVER THE LAST MILE OF CUDA. With features: intellisense-friendly, structured launch, automatic cuda graph generation and updating.")
    set_license("Apache-2.0")

    add_urls("https://github.com/MuGdxy/muda.git")

    add_versions("2025.02.01", "008b6fdd48e6ffa7bcaf79943beb24f940d8da93")

    set_policy("package.install_locally", true)

    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr")

    on_install(function (package)
        os.cp("src/muda", package:installdir("include"))
    end)
