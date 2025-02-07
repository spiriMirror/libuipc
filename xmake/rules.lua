rule("component")
    on_load(function (target)
        target:set("kind", "shared")
        target:add("includedirs", path.join(os.projectdir(), "include"), {public = true})
        if target:is_plat("windows") then
            target:add("defines", format("UIPC_%s_EXPORT_DLL", target:name():upper()))
        end
    end)
