#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <algorithm>
#include <cmath>

namespace
{
struct ReboundResult
{
    uipc::Float contact_y       = 0.0;
    uipc::Float rebound_peak_y  = 0.0;
    uipc::Float rebound_height  = 0.0;
    bool        touched_ground  = false;
    bool        rebounded       = false;
};

ReboundResult run_drop_case(const std::string& output_path, const std::string& integrator_type)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["dt"]                            = 0.01;
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["integrator"]["type"]            = integrator_type;
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["contact"]["constitution"]       = "ipc";
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        AffineBodyConstitution abd;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);

        auto obj = scene.objects().create("falling_abd");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 1, 0},
                               Vector3{0, 0, 1},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(Vs.begin(),
                       Vs.end(),
                       Vs.begin(),
                       [](const Vector3& v) { return v * 0.3; });

        auto tet = tetmesh(Vs, Ts);
        label_surface(tet);
        label_triangle_orient(tet);
        abd.apply_to(tet, 100.0_MPa);

        auto trans = view(tet.transforms());
        trans[0](1, 3) += 0.6;  // start above the ground

        obj->geometries().create(tet);
        obj->geometries().create(ground(0.0));
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    auto abd_accessor = world.features().find<AffineBodyStateAccessorFeature>();
    REQUIRE(abd_accessor != nullptr);
    REQUIRE(abd_accessor->body_count() == 1);

    SimplicialComplex abd_state = abd_accessor->create_geometry();
    abd_state.instances().create<Matrix4x4>(builtin::transform);

    vector<Float> ys;
    ys.reserve(300);

    auto sample_y = [&]() -> Float {
        abd_accessor->copy_to(abd_state);
        return abd_state.transforms().view()[0](1, 3);
    };

    world.retrieve();
    ys.push_back(sample_y());

    while(world.frame() < 220)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        ys.push_back(sample_y());
    }

    ReboundResult result;

    if(ys.size() < 4)
        return result;

    SizeT contact_idx = 0;
    bool  found_contact = false;
    for(SizeT i = 1; i + 1 < ys.size(); ++i)
    {
        if(ys[i] < ys[i - 1] && ys[i + 1] > ys[i])
        {
            contact_idx   = i;
            found_contact = true;
            break;
        }
    }

    if(!found_contact)
        return result;

    result.touched_ground = true;
    result.contact_y      = ys[contact_idx];

    auto peak_it = std::max_element(ys.begin() + contact_idx + 1, ys.end());
    if(peak_it == ys.end())
        return result;

    result.rebound_peak_y = *peak_it;
    result.rebound_height = result.rebound_peak_y - result.contact_y;
    result.rebounded      = result.rebound_height > 0.0;
    return result;
}
}  // namespace

TEST_CASE("79_abd_bdf2_rebound_higher_than_bdf1", "[abd][bdf]")
{
    using namespace uipc;

    auto output_root = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    auto bdf1 = run_drop_case(fmt::format("{}bdf1/", output_root), "bdf1");
    auto bdf2 = run_drop_case(fmt::format("{}bdf2/", output_root), "bdf2");

    REQUIRE(bdf1.touched_ground);
    REQUIRE(bdf2.touched_ground);
    REQUIRE(bdf1.rebounded);
    REQUIRE(bdf2.rebounded);

    INFO("bdf1 contact_y=" << bdf1.contact_y << ", rebound_peak_y=" << bdf1.rebound_peak_y
                           << ", rebound_height=" << bdf1.rebound_height);
    INFO("bdf2 contact_y=" << bdf2.contact_y << ", rebound_peak_y=" << bdf2.rebound_peak_y
                           << ", rebound_height=" << bdf2.rebound_height);

    REQUIRE(bdf2.rebound_height > bdf1.rebound_height);
}
