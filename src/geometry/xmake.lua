add_requires("libigl", "octree", "tetgen")

target("uipc_geometry")
    add_rules("component")
    add_files("**.cpp")
    add_includedirs(os.scriptdir())
    add_headerfiles(
        path.join(os.projectdir(), "include/uipc/geometry/**.h"),
        path.join(os.projectdir(), "include/uipc/geometry/**.inl"),
        "**.hpp"
    )
    add_defines()
    add_deps("uipc_core")
    add_packages("octree", "tetgen")
    add_packages("libigl", {public = true})
