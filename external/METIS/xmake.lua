target("metis")
    set_kind("static")
    set_languages("c11")
    -- Need PIC since this static lib is linked into shared libraries
    add_cflags("-fPIC", {tools = {"gcc", "clang"}})
    add_files("libmetis/*.c")
    add_includedirs("include", {public = true})
    add_deps("GKlib")
    -- suppress warnings from legacy C code
    if is_plat("windows") then
        add_cflags("/wd4005", "/wd4244", "/wd4267", "/wd4996")
    end
