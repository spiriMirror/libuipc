target("backend_cuda")
    set_group("tests")
    add_rules("uipc_test")
    add_deps("cuda")
    add_files("**.cu", "**.cpp")
    -- parity with CMakeLists.txt: the tests under internals/ link against
    -- symbols of the CUDA backend library itself, which the backend DLL on
    -- Windows does not export, so Windows builds leave them out.
    if is_plat("windows") then
        remove_files("internals/**")
    end
    if has_config("dev") then
        add_rules("clangd")
    end
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_cuflags("/wd4819", {tools = "cl"})
    -- parity with apps/tests/backends/cuda/CMakeLists.txt: the test .cu files
    -- use cuda_tool launch/dense math (needs extended lambdas + relaxed
    -- constexpr), and CUDA >= 13 CCCL requires the conforming MSVC preprocessor.
    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr")
    if is_plat("windows") then
        add_cuflags("-Xcompiler=/Zc:preprocessor")
    end
