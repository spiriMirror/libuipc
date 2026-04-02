#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/strain_plastic_discrete_shell_bending.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

constexpr SizeT SheetResolution = 21;
constexpr Float SheetSize       = 1.0;

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
}  // namespace

TEST_CASE("82_discrete_shell_bending_strain_plastic_open_strip", "[fem][strain_plastic_dsb]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, 0, 0};
    config["contact"]["enable"]             = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["dt"]                            = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    auto object = scene.objects().create("strip");

    NeoHookeanShell            nhs;
    StrainPlasticDiscreteShellBending pdsb;
    SoftPositionConstraint     spc;

    auto patch = load_center_patch();
    auto moduli         = ElasticModuli2D::youngs_poisson(10.0_MPa, 0.49);
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

            const Float t = static_cast<Float>(std::min<SizeT>(info.frame(), 60)) / 60.0;
            const Float y = amplitude * std::sin(std::numbers::pi_v<Float> * t);
            aim_position_view[moving_vertex] =
                rest_positions[moving_vertex] + Vector3::UnitY() * y;
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 70)
    {
        world.advance();
        REQUIRE(world.is_valid());

        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
        auto sc = slot->geometry().as<SimplicialComplex>();
        const auto position_view = view(sc->positions());
        REQUIRE(std::ranges::all_of(position_view,
                                    [](const Vector3& p)
                                    {
                                        return std::isfinite(p.x()) && std::isfinite(p.y())
                                               && std::isfinite(p.z());
                                    }));
    }
}
