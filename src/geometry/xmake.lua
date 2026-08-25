-- pin octree to v2.5: v3.0+ moved the headers to an `orthotree/` subdirectory
-- (and changed the API), which breaks `src/geometry/octree.cpp`'s flat includes;
-- v2.5 keeps the flat octree.h + adaptor.eigen.h layout this code expects
add_requires("libigl", "octree v2.5", "tbb")

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
    add_deps("uipc_core", "metis")
    if has_config("vdb") then
        add_deps("uipc_vdb")
    end
    add_packages("octree", "tbb")
    add_packages("libigl", {public = true})
