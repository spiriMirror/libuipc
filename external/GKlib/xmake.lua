target("GKlib")
    set_kind("static")
    set_languages("c11")
    add_files("src/*.c")
    if is_plat("windows") then
        add_files("src/win32/adapt.c")
    end
    add_includedirs("include", {public = true})
    if is_plat("windows") then
        add_includedirs("include/win32", {public = true})
    end
    if is_plat("linux") then
        add_defines("LINUX", {public = true})
        add_syslinks("m")
    end
    add_defines("NDEBUG", {public = true})
    -- suppress warnings from legacy C code
    if is_plat("windows") then
        add_cflags("/wd4005", "/wd4244", "/wd4267", "/wd4996")
    end
