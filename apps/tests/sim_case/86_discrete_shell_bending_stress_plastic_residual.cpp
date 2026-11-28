#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/stress_plastic_discrete_shell_bending.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <algorithm>
#include <cmath>

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
    SizeT             v00       = 0;
    SizeT             v10       = 0;
    SizeT             v11       = 0;
    SizeT             v01       = 0;
    Float             cell_span = 0.0;
};

ClothPatch load_center_patch()
{
    vector<Vector3>  Vs;
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

    return ClothPatch{std::move(mesh), std::move(Vs), v00, v10, v11, v01, step};
}

Float center_hinge_angle(const SimplicialComplex& sc, const ClothPatch& patch)
{
    auto          ps = sc.positions().view();
    const Vector3 n0 =
        (ps[patch.v10] - ps[patch.v00]).cross(ps[patch.v11] - ps[patch.v00]);
    const Vector3 n1 =
        (ps[patch.v00] - ps[patch.v01]).cross(ps[patch.v11] - ps[patch.v01]);
    REQUIRE(n0.norm() > 0);
    REQUIRE(n1.norm() > 0);
    return std::atan2(n0.cross(n1).norm(), n0.dot(n1));
}

vector<Float> residual_hinge_angles(bool use_stress_plastic)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    output_path += use_stress_plastic ? "stress_plastic/" : "elastic/";

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
    auto object = scene.objects().create(use_stress_plastic ? "stress_plastic_strip" :
                                                              "elastic_strip");

    auto patch = load_center_patch();

    NeoHookeanShell        nhs;
    SoftPositionConstraint spc;
    auto moduli = ElasticModuli2D::youngs_poisson(100.0_kPa, 0.49);
    nhs.apply_to(patch.mesh, moduli);
    const Float kappa = DiscreteShellBending::bending_stiffness(100.0_kPa, 0.49, 0.001_m);

    if(use_stress_plastic)
    {
        StressPlasticDiscreteShellBending spdsb;
        spdsb.apply_to(patch.mesh, kappa, 0.36 * kappa, 0.0);
    }
    else
    {
        DiscreteShellBending dsb;
        dsb.apply_to(patch.mesh, kappa);
    }

    spc.apply_to(patch.mesh, 100.0);

    auto [slot, rest_slot] = object->geometries().create(patch.mesh);
    (void)rest_slot;

    constexpr SizeT RampFrames    = 20;
    constexpr SizeT HoldFrames    = 10;
    constexpr SizeT ReleaseFrames = 44;
    const Float     TargetLift    = 5.0 * patch.cell_span;

    scene.animator().insert(
        *object,
        [rest_positions = std::move(patch.rest_positions),
         moving_vertex  = patch.v01,
         target_lift    = TargetLift](Animation::UpdateInfo& info)
        {
            auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();

            auto is_constrained = geo->vertices().find<IndexT>(builtin::is_constrained);
            auto aim_position = geo->vertices().find<Vector3>(builtin::aim_position);
            auto is_constrained_view = view(*is_constrained);
            auto aim_position_view   = view(*aim_position);

            std::ranges::fill(is_constrained_view, 1);
            std::copy(rest_positions.begin(),
                      rest_positions.end(),
                      aim_position_view.begin());

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

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    const SizeT   end_frame = RampFrames + HoldFrames + ReleaseFrames;
    vector<Float> residuals;
    while(world.frame() < end_frame)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
        if(world.frame() > end_frame - ReleaseFrames / 4)
            residuals.push_back(
                center_hinge_angle(*slot->geometry().as<SimplicialComplex>(), patch));
    }
    return residuals;
}
}  // namespace

TEST_CASE("86_discrete_shell_bending_stress_plastic_residual", "[fem][stress_plastic_dsb]")
{
    const auto elastic = residual_hinge_angles(false);
    const auto plastic = residual_hinge_angles(true);
    // The surrounding pinned sheet resists the crease. Its residual should
    // remain distinguishable from the elastic oscillation over the unloading tail.
    CHECK(std::ranges::max(elastic) < 1.0e-2);
    CHECK(std::ranges::min(plastic) > std::ranges::max(elastic));
}

TEST_CASE("stress plastic hinge unloads to its predicted rest angle", "[fem][stress_plastic_dsb]")
{
    const Float     target      = GENERATE(0.05, 0.4);
    constexpr Float a           = 0.025;
    constexpr Float yield_angle = 0.1;
    const vector<Vector3> vertices{{-a, 0, 0}, {a, 0, 0}, {0, 0, a}, {0, 0, -a}};
    const vector<Vector3i> triangles{{0, 1, 2}, {1, 0, 3}};
    auto                   mesh = trimesh(vertices, triangles);
    label_surface(mesh);
    NeoHookeanShell{}.apply_to(mesh, ElasticModuli2D::youngs_poisson(100.0_kPa, 0.49));
    const Float kappa = DiscreteShellBending::bending_stiffness(100.0_kPa, 0.49, 0.001_m);
    // L0=2a, the adjacent rest areas sum to 2a², and h_bar=A/(3L0)=a/3.
    const Float yield_moment = 2 * kappa * (2 * a) / (a / 3) * yield_angle;
    StressPlasticDiscreteShellBending{}.apply_to(mesh, kappa, yield_moment, 0.0);
    SoftPositionConstraint{}.apply_to(mesh, 100.0);
    auto fixed = view(*mesh.vertices().find<IndexT>(builtin::is_fixed));
    std::fill_n(fixed.begin(), 3, 1);
    auto dynamic = view(*mesh.vertices().find<IndexT>(builtin::is_dynamic));
    std::ranges::fill(dynamic, 0);  // Suppress inertial overshoot during unloading.

    Engine engine{"cuda", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE) + "single_hinge/"};
    World world{engine};
    auto  config                                = test::Scene::default_config();
    config["dt"]                                = 0.01;
    config["gravity"]                           = Vector3::Zero().eval();
    config["contact"]["enable"]                 = false;
    config["newton"]["velocity_tol"]            = 1e-5;
    config["newton"]["semi_implicit"]["enable"] = 0;
    config["linear_system"]["tol_rate"]         = 1e-6;
    Scene scene{config};
    auto  obj         = scene.objects().create("single_hinge");
    auto [slot, rest] = obj->geometries().create(mesh);
    scene.animator().insert(
        *obj,
        [target](Animation::UpdateInfo& info)
        {
            auto sc = info.geo_slots()[0]->geometry().as<SimplicialComplex>();
            view(*sc->vertices().find<IndexT>(builtin::is_constrained))[3] =
                info.frame() <= 30;
            const Float angle = target * std::min<Float>(info.frame() / 20.0, 1.0);
            view(*sc->vertices().find<Vector3>(builtin::aim_position))[3] =
                Vector3{0, a * std::sin(angle), -a * std::cos(angle)};
        });
    world.init(scene);
    REQUIRE(world.is_valid());
    Float peak = 0, angle = 0;
    for(SizeT frame = 1; frame <= 74; ++frame)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        const auto& tip =
            slot->geometry().as<SimplicialComplex>()->positions().view()[3];
        angle = std::atan2(tip.y(), -tip.z());
        if(frame <= 30)
            peak = std::max(peak, angle);
    }
    REQUIRE((peak > yield_angle) == (target > yield_angle));
    CHECK(angle == Catch::Approx(std::max(peak - yield_angle, 0.0)).margin(1e-4));
}
