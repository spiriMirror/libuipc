#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/strain_plastic_discrete_shell_bending.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <algorithm>
#include <filesystem>
#include <numbers>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

constexpr SizeT EndFrame     = 60;
constexpr SizeT RecoverFrame = 40;
constexpr std::size_t DumpMagic = 0xc2663291fdf3;
constexpr SizeT SheetResolution = 21;
constexpr Float SheetSize       = 1.0;

struct SimulationResult
{
    vector<Vector3> final_positions;
    bool            recover_result = false;
};

struct ClothPatch
{
    SimplicialComplex mesh;
    vector<Vector3>   rest_positions;
    SizeT             v00 = 0;
    SizeT             v10 = 0;
    SizeT             v11 = 0;
    SizeT             v01 = 0;
    Float             cell_span = 0.0;
};

ClothPatch load_center_patch()
{
    vector<Vector3> Vs;
    vector<Vector3i> Fs;
    Vs.reserve(SheetResolution * SheetResolution);
    Fs.reserve((SheetResolution - 1) * (SheetResolution - 1) * 2);

    const Float half_size = 0.5 * SheetSize;
    const Float step      = SheetSize / static_cast<Float>(SheetResolution - 1);

    for(SizeT j = 0; j < SheetResolution; ++j)
    {
        const Float z = -half_size + step * static_cast<Float>(j);
        for(SizeT i = 0; i < SheetResolution; ++i)
        {
            const Float x = -half_size + step * static_cast<Float>(i);
            Vs.emplace_back(x, 0.0, z);
        }
    }

    for(SizeT j = 0; j + 1 < SheetResolution; ++j)
    {
        for(SizeT i = 0; i + 1 < SheetResolution; ++i)
        {
            const SizeT v00 = j * SheetResolution + i;
            const SizeT v10 = v00 + 1;
            const SizeT v01 = v00 + SheetResolution;
            const SizeT v11 = v01 + 1;
            Fs.emplace_back(v00, v11, v10);
            Fs.emplace_back(v00, v01, v11);
        }
    }

    auto mesh = trimesh(Vs, Fs);
    label_surface(mesh);

    const SizeT cell_i = SheetResolution / 2 - 1;
    const SizeT cell_j = SheetResolution / 2 - 1;
    const SizeT v00    = cell_j * SheetResolution + cell_i;
    const SizeT v10    = v00 + 1;
    const SizeT v01    = v00 + SheetResolution;
    const SizeT v11    = v01 + 1;

    return ClothPatch{std::move(mesh),
                      std::move(Vs),
                      v00,
                      v10,
                      v11,
                      v01,
                      step};
}

S<geometry::GeometrySlot> build_scene(Scene& scene)
{
    NeoHookeanShell             nhs;
    StrainPlasticDiscreteShellBending pdsb;
    SoftPositionConstraint      spc;

    auto object = scene.objects().create("strip");
    auto patch  = load_center_patch();

    auto moduli = ElasticModuli2D::youngs_poisson(10.0_MPa, 0.49);
    nhs.apply_to(patch.mesh, moduli);
    pdsb.apply_to(patch.mesh, 5.0_kPa, 0.02, 0.0);
    spc.apply_to(patch.mesh, 100.0);

    auto [slot, rest_slot] = object->geometries().create(patch.mesh);
    (void)rest_slot;

    scene.animator().insert(
        *object,
        [rest_positions = std::move(patch.rest_positions),
         moving_vertex = patch.v01,
         amplitude     = 4.0 * patch.cell_span](Animation::UpdateInfo& info)
        {
            auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();

            auto is_constrained = geo->vertices().find<IndexT>(builtin::is_constrained);
            auto aim_position = geo->vertices().find<Vector3>(builtin::aim_position);
            auto is_constrained_view = view(*is_constrained);
            auto aim_position_view   = view(*aim_position);

            std::ranges::fill(is_constrained_view, 1);
            std::copy(rest_positions.begin(), rest_positions.end(), aim_position_view.begin());

            const Float normalized_frame =
                static_cast<Float>(std::min<SizeT>(info.frame(), EndFrame)) / EndFrame;
            const Float y = amplitude * std::sin(std::numbers::pi_v<Float> * normalized_frame);
            aim_position_view[moving_vertex] =
                rest_positions[moving_vertex] + Vector3::UnitY() * y;
        });

    return slot;
}

SimulationResult run_case(const std::filesystem::path& workspace,
                          bool                        dump_frames,
                          std::optional<SizeT>        recover_frame = std::nullopt)
{
    Engine engine{"cuda", workspace.string()};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, 0, 0};
    config["contact"]["enable"]             = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["dt"]                            = 0.01;
    test::Scene::dump_config(config, workspace.string());

    Scene           scene{config};
    auto slot = build_scene(scene);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", workspace.string(), world.frame()));

    SimulationResult result;
    if(recover_frame)
    {
        result.recover_result = world.recover(*recover_frame);
        if(!result.recover_result)
            return result;

        REQUIRE(world.frame() == *recover_frame);
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", workspace.string(), world.frame()));
    }

    while(world.frame() < EndFrame)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", workspace.string(), world.frame()));
        if(dump_frames)
            world.dump();
    }

    auto sc = slot->geometry().as<SimplicialComplex>();
    const auto position_view = view(sc->positions());
    result.final_positions.assign(position_view.begin(), position_view.end());
    return result;
}

void clear_dump_dir(const std::filesystem::path& workspace)
{
    std::filesystem::remove_all(workspace / "dump");
}

Float max_position_diff(span<const Vector3> a, span<const Vector3> b)
{
    REQUIRE(a.size() == b.size());

    Float max_diff = 0.0;
    for(SizeT i = 0; i < a.size(); ++i)
    {
        max_diff = std::max(max_diff, (a[i] - b[i]).norm());
    }
    return max_diff;
}
}  // namespace

TEST_CASE("78_strain_plastic_discrete_shell_bending_recover", "[fem][strain_plastic_dsb][recover]")
{
    namespace fs = std::filesystem;

    auto workspace = fs::path(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));
    clear_dump_dir(workspace);

    SECTION("recover_continues_from_dumped_state")
    {
        auto baseline = run_case(workspace, true);
        auto resumed  = run_case(workspace, false, RecoverFrame);

        REQUIRE(resumed.recover_result);
        REQUIRE(baseline.final_positions.size() == resumed.final_positions.size());
        CHECK(max_position_diff(baseline.final_positions, resumed.final_positions)
              < 1e-6);
    }

}
