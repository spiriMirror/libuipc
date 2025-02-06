add_requires("libigl", "octree", "tetgen")

target("geometry")
    add_rules("component")
    add_files("**.cpp")
    add_includedirs(os.scriptdir())
    add_headerfiles(
        path.join(os.projectdir(), "include/uipc/geometry/**.h"),
        path.join(os.projectdir(), "include/uipc/geometry/**.inl"),
        "**.hpp"
    )

    add_deps("core")
    add_packages("octree", "tetgen")
    add_packages("libigl", {public = true})

package("tetgen")
    add_urls("https://github.com/spiriMirror/tetgen.git")
    add_versions("1.6.0", "4f3bfba3997f20aa1f96cfaff604313a8c2c85b6")

    add_configs("shared", {description = "Build shared library.", default = false, type = "boolean", readonly = true})

    set_policy("package.install_locally", true)

    on_install(function (package)
        io.writefile("xmake.lua", [[
            add_rules("mode.debug", "mode.release")
            set_languages("c++11")
            target("tetgen")
                set_kind("$(kind)")
                add_files("tetgen.cxx", "predicates.cxx")
                add_headerfiles("tetgen.h", {prefixdir = "tetgen"})
                add_defines("TETLIBRARY")
        ]])
        import("package.tools.xmake").install(package)
    end)

    on_test(function (package)
        assert(package:has_cxxfuncs("tetrahedralize", {includes = "tetgen/tetgen.h"}))
    end)

package("octree")
    set_kind("library", {headeronly = true})
    set_homepage("https://github.com/attcs/Octree")
    set_description("Octree/Quadtree/N-dimensional linear tree")
    set_license("MIT")

    set_urls("https://github.com/attcs/Octree/archive/refs/tags/$(version).tar.gz",
             "https://github.com/attcs/Octree.git", {submodules = false})

    add_versions("v2.5", "86088cd000254aeddf4f9d75c0600b7f799e062340394124d69760829ed317fe")

    set_policy("package.install_locally", true)

    on_check(function (package)
        if not package:is_arch("x64", "x86", "x86_64") then
            raise("package(octree) only support x86 arch")
        end

        local msvc = package:toolchain("msvc")
        if package:is_arch("x64") and msvc then
            local vs_toolset = msvc:config("vs_toolset")
            if vs_toolset then
                local vs_toolset_ver = import("core.base.semver").new(vs_toolset)
                local minor = vs_toolset_ver:minor()
                assert(minor and minor >= 30, "package(octree) require vs_toolset >= 14.3")
            end
        end
    end)

    on_install(function (package)
        os.cp("*.h", package:installdir("include/Octree"))
    end)

    on_test(function (package)
        assert(package:check_cxxsnippets({test = [[
            using namespace OrthoTree;
            void test() {
                auto constexpr points = std::array{ Point3D{0,0,0}, Point3D{1,1,1}, Point3D{2,2,2} };
                auto const octree = OctreePointC(points, 3 /*max depth*/);

                auto const searchBox = BoundingBox3D{ {0.5, 0.5, 0.5}, {2.5, 2.5, 2.5} };
                auto const pointIDs = octree.RangeSearch(searchBox); //: { 1, 2 }

                auto neighborNo = 2;
                auto pointIDsByKNN = octree.GetNearestNeighbors(Point3D{ 1.1, 1.1, 1.1 }
                    , neighborNo
                ); //: { 1, 2 }
            }
        ]]}, {configs = {languages = "c++20"}, includes = "Octree/octree.h"}))
    end)
