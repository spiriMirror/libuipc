target("GKlib")
    set_kind("static")
    set_languages("c11")
    -- Need PIC since this static lib is linked into shared libraries
    add_cflags("-fPIC", {tools = {"gcc", "clang"}})
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
    if is_plat("windows") then
        -- MSVC uses __declspec(thread) instead of __thread for TLS
        add_defines("__thread=__declspec(thread)", {public = true})
        -- Windows has no POSIX regex, use GKlib's built-in implementation
        add_defines("USE_GKREGEX", {public = true})
    end
