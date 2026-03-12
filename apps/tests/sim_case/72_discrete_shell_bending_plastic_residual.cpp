#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/plastic_discrete_shell_bending.h>
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
            Fs.emplace_back(v00, v10, v11);
            Fs.emplace_back(v00, v11, v01);
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

uipc::Float residual_hinge_angle(bool use_plastic)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    output_path += use_plastic ? "plastic/" : "elastic/";

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                         = test::Scene::default_config();
    config["gravity"]                   = Vector3{0, 0, 0};
    config["contact"]["enable"]         = false;
    config["line_search"]["max_iter"]   = 8;
    config["linear_system"]["tol_rate"] = 1e-3;
    config["dt"]                        = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    auto  object = scene.objects().create(use_plastic ? "plastic_strip" : "elastic_strip");

    auto patch = load_center_patch();

    NeoHookeanShell        nhs;
    SoftPositionConstraint spc;
    auto                   moduli = ElasticModuli2D::youngs_poisson(100.0_kPa, 0.49);
    nhs.apply_to(patch.mesh, moduli);

    if(use_plastic)
    {
        PlasticDiscreteShellBending pdsb;
        pdsb.apply_to(patch.mesh, 20.0_kPa, 0.03, 0.0);
    }
    else
    {
        DiscreteShellBending dsb;
        dsb.apply_to(patch.mesh, 20.0_kPa);
    }

    spc.apply_to(patch.mesh, 100.0);

    auto [slot, rest_slot] = object->geometries().create(patch.mesh);
    (void)rest_slot;

    constexpr SizeT RampFrames    = 60;
    constexpr SizeT HoldFrames    = 40;
    constexpr SizeT ReleaseFrames = 180;
    const Float     TargetLift    = 5.0 * patch.cell_span;

    scene.animator().insert(
        *object,
        [rest_positions = std::move(patch.rest_positions),
         moving_vertex = patch.v01,
         target_lift   = TargetLift](Animation::UpdateInfo& info)
        {
            auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();

            auto is_constrained      = geo->vertices().find<IndexT>(builtin::is_constrained);
            auto aim_position        = geo->vertices().find<Vector3>(builtin::aim_position);
            auto is_constrained_view = view(*is_constrained);
            auto aim_position_view   = view(*aim_position);

            std::ranges::fill(is_constrained_view, 1);
            std::copy(rest_positions.begin(), rest_positions.end(), aim_position_view.begin());

            const SizeT frame = info.frame();
            if(frame <= RampFrames + HoldFrames)
            {
                Float lift = target_lift;
                if(frame < RampFrames)
                    lift = target_lift * static_cast<Float>(frame)
                           / static_cast<Float>(RampFrames);

                aim_position_view[moving_vertex] =
                    rest_positions[moving_vertex] + Vector3::UnitY() * lift;
            }
            else
            {
                is_constrained_view[moving_vertex] = 0;
            }
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    const SizeT end_frame = RampFrames + HoldFrames + ReleaseFrames;
    while(world.frame() < end_frame)
    {
        world.advance();
        REQUIRE(world.is_valid());
    }

    world.retrieve();

    auto sc = slot->geometry().as<SimplicialComplex>();
    auto ps = view(sc->positions());

    const Vector3 x0 = ps[patch.v00];
    const Vector3 x1 = ps[patch.v10];
    const Vector3 x2 = ps[patch.v11];
    const Vector3 x3 = ps[patch.v01];

    const Vector3 n0 = (x1 - x0).cross(x2 - x0);
    const Vector3 n1 = (x2 - x3).cross(x0 - x3);
    const Float   n0_norm = n0.norm();
    const Float   n1_norm = n1.norm();
    REQUIRE(n0_norm > 0.0);
    REQUIRE(n1_norm > 0.0);

    const Float cos_theta =
        std::clamp(n0.dot(n1) / (n0_norm * n1_norm), Float(-1.0), Float(1.0));
    const Float triangle_normal_angle = std::acos(cos_theta);

    return std::abs(std::numbers::pi_v<Float> - triangle_normal_angle);
}
}  // namespace

TEST_CASE("72_discrete_shell_bending_plastic_residual", "[fem][plastic_dsb]")
{
    const auto elastic_angle = residual_hinge_angle(false);
    const auto plastic_angle = residual_hinge_angle(true);

    CHECK(elastic_angle < 1.0e-2);
    CHECK(plastic_angle > 5.0e-2);
    CHECK(plastic_angle > elastic_angle + 5.0e-2);
}
