if has_config("backend_cuda") then
    includes("cuda")
end

target("backend_common")
    add_files("common/**.cpp")
    add_headerfiles("common/**.h", "common/**.inl")
    add_includedirs(os.scriptdir(), {public = true})
    add_deps("uipc_core", {public = true})

target("none")
    add_rules("backend")
    add_files("none/*.cpp")
    add_headerfiles("none/*.h")

rule("backend")
    on_load(function (target)
        print("Adding backend:", target:name())

        target:set("basename", "uipc_backend_" .. target:name())

        target:set("kind", "shared")
        target:add("files", path.join(os.scriptdir(), "base_source/*.cpp"))

        target:add("includedirs",
            path.join(os.scriptdir(), target:name())
        )

        local format_string = [[%s=R"(%s)"]]
        target:add("defines",
            "UIPC_BACKEND_EXPORT_DLL",
            format(format_string, "UIPC_BACKEND_DIR", os.scriptdir()),
            format(format_string, "UIPC_BACKEND_NAME", target:name())
        )

        target:add("deps", "backend_common", {public = false})
    end)