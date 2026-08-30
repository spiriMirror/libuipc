if has_config("backend_cuda") then
    includes("cuda")
end

target("none")
    add_rules("backend")
    add_files("none/*.cpp")
    add_headerfiles("none/*.h")

rule("backend")
    on_load(function (target)
        import("core.base.semver")
        print("Adding backend:", target:name())

        target:set("basename", "uipc_backend_" .. target:name())

        target:set("kind", "shared")
        target:add("files", path.join(os.scriptdir(), "common/*.cpp"))
        target:add("headerfiles", "common/*.h", "common/details/*.h", "common/details/*.inl")
        target:add("includedirs",
            path.directory(os.scriptdir()),
            path.join(os.scriptdir(), target:name())
        )

        local format_string = [[%s=R"(%s)"]]
        local version = semver.new(target:version())
        target:add("defines",
            "UIPC_BACKEND_EXPORT_DLL",
            format(format_string, "UIPC_BACKEND_DIR", os.scriptdir()),
            format(format_string, "UIPC_BACKEND_NAME", target:name()),
            "UIPC_VERSION_MAJOR=" .. version:major(),
            "UIPC_VERSION_MINOR=" .. version:minor(),
            "UIPC_VERSION_PATCH=" .. version:patch()
        )

        target:add("deps", "uipc_core")
    end)
