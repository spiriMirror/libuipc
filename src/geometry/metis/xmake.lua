target("uipc_metis")
    set_kind("static")
    set_group("components")
    set_languages("c++20")
    add_files("*.cpp")
    add_includedirs(".", {public = true})
    add_defines("NDEBUG")

    if is_plat("linux") then
        add_defines("LINUX")
        add_syslinks("m")
        add_cxflags("-fPIC", {tools = {"gcc", "clang"}})
    end
