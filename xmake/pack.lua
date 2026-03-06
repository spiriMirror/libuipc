includes("@builtin/xpack")

xpack("pyuipc")
    set_formats("zip")
    set_extension(".whl")

    add_targets("pyuipc")

    on_package(function (package)
        import("target.action.install", {alias = "_do_install_target"})
        import("lib.detect.find_tool")
        import("core.project.config")
        import("core.base.semver")
        import("helper")
        -- Copy project file
        local build_dir = path.absolute(package:builddir())
        os.tryrm(build_dir)
        os.mkdir(build_dir)
        os.vcp(path.join(os.projectdir(), "python/*"), build_dir)

        local pyuipc_target = package:target("pyuipc")
        local modules_dir = path.join(build_dir, "src/uipc/modules", config.get("mode"))
        os.mkdir(modules_dir)
        -- Copy project shared libraries
        _do_install_target(pyuipc_target, {
            headers = false,
            binaries = false,
            libraries = true,
            packages = true,
            installdir = modules_dir,
            libdir = "",
            bindir = "",
        })
        os.rm(path.join(modules_dir, "*.lib"))

        -- Build stub file
        local python
        local python_pkg = pyuipc_target:pkg("python")
        if python_pkg then
            local installdir = python_pkg:installdir()
            if installdir then
                python = path.join(installdir, "bin/python")
            end
        end
        python = python or "python"

        local ok = try { function()
            os.vrunv(python, { "-c", "import mypy" })
            return true
        end }

        local uv = assert(find_tool("uv"), "uv not found!")
        if not ok then
            os.vrunv(uv.program, {"pip", "install", "mypy", "numpy"})
        end

        local LD_LIBRARY_PATH = path.splitenv(os.getenv("LD_LIBRARY_PATH") or "")
        table.insert(LD_LIBRARY_PATH, 1, modules_dir)
        local python_libdir = try { function()
            local out = os.iorunv(python, {"-c", "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"})
            return out:trim()
        end }
        if python_libdir then
            table.insert(LD_LIBRARY_PATH, python_libdir)
        end
        os.vrunv(python, {
            path.join(os.projectdir(), "scripts/stubgen.py"),
            "--source_dir=" .. path.join(build_dir, "src"),
            "--output_dir=" .. path.join(build_dir, "src"),
            "--build_type=" .. config.get("mode"),
        }, {setenvs = {["LD_LIBRARY_PATH"] = path.joinenv(LD_LIBRARY_PATH)}})

        -- Build .whl file
        local cuda_version = helper.get_cuda_version()
        local python_version = semver.new(helper.get_python_version(pyuipc_target))
        local git_rev = try { function()
            return os.iorunv("git", {"rev-parse", "--short", "HEAD"})
        end }
        if git_rev then
            git_rev = git_rev:trim()
        end

        local argv = {
            "build",
            "--wheel", -- don't package tarball
            "--python", format("%s.%s", python_version:major(), python_version:minor()),
            "--out-dir", path.absolute(package:outputdir())
        }

        local opt = {
            curdir = build_dir,
            envs = {
                CUDA_VERSION = helper.format_version_with_prefix("cu", cuda_version),
                BUILD_MODE = config.get("mode"),
                GIT_REV = git_rev,
            },
        }

        os.vrunv(uv.program, argv, opt)
    end)