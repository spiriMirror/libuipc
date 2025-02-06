if get_config("backend") == "cuda" then
    includes("cuda")
end

target("none")
    add_rules("backend")
    add_files("none/*.cpp")
    add_headerfiles("none/*.h")

rule("backend")
    on_load(function (target)
        print("Adding backend:", target:name())

        target:set("basename", "uipc_backend_" .. target:name())

        target:set("kind", "shared")
        target:add("files", path.join(os.scriptdir(), "common/*.cpp"))
        target:add("headefiles", "common/*.h", "common/details/*.inl")
        target:add("includedirs",
            path.directory(os.scriptdir()),
            path.join(os.scriptdir(), target:name())
        )

        local format_string = [[%s=R"(%s)"]]
        target:add("defines",
            "UIPC_BACKEND_EXPORT_DLL",
            format(format_string, "UIPC_BACKEND_DIR", path.unix(path.join(os.scriptdir(), target:name()))),
            format(format_string, "UIPC_BACKEND_NAME", target:name())
        )

        target:add("deps", "core")
    end)
