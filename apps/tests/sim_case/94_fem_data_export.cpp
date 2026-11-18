#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/core/fem_system_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <uipc/builtin/attribute_name.h>
#include <algorithm>

using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

namespace
{
Matrix3x3 Ds(const Vector3& x0, const Vector3& x1, const Vector3& x2, const Vector3& x3)
{
    Matrix3x3 Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Ds.col(2) = x3 - x0;
    return Ds;
}

// Central FD derivative of the exported energy w.r.t. one vertex coordinate.
template <typename EnergyFn>
Float fd_energy_derivative(EnergyFn&& energy_at, vector<Vector3> positions, IndexT vertex, IndexT dim, Float h)
{
    positions[vertex][dim] += h;
    Float e_plus = energy_at(positions);
    positions[vertex][dim] -= 2 * h;
    Float e_minus = energy_at(positions);
    return (e_plus - e_minus) / (2 * h);
}

// Central FD column of an exported Hessian block: d(grad_row)/d(x_col[s]).
template <typename GradientFn>
Vector3 fd_gradient_derivative(GradientFn&&    gradient_at,
                               vector<Vector3> positions,
                               IndexT          row,
                               IndexT          col,
                               IndexT          s,
                               Float           h)
{
    positions[col][s] += h;
    vector<Vector3> g_plus = gradient_at(positions);
    positions[col][s] -= 2 * h;
    vector<Vector3> g_minus = gradient_at(positions);
    return (g_plus[row] - g_minus[row]) / (2 * h);
}
}  // namespace

TEST_CASE("94_fem_data_export_snh", "[fem]")
{
    const vector<Vector3>  Vs = {Vector3{0, 0, 1},
                                 Vector3{0, -1, 0},
                                 Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                 Vector3{std::sqrt(3) / 2, 0, -0.5}};
    const vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    const Matrix3x3 DmInv = Ds(Vs[0], Vs[1], Vs[2], Vs[3]).inverse();

    std::string this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    const Float dt                    = 0.01;  // must match the config below
    auto        config                = Scene::default_config();
    config["gravity"]                 = Vector3{0, -9.8, 0};
    config["dt"]                      = 0.01_s;
    config["contact"]["enable"]       = false;
    config["line_search"]["max_iter"] = 8;

    Scene scene{config};

    StableNeoHookean snh;
    auto             mesh = tetmesh(Vs, Ts);
    snh.apply_to(mesh, ElasticModuli::youngs_poisson(1.0e4, 0.3), 1e3);

    // keep vertex 0 fixed so gravity actually deforms the tet
    auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
    auto is_fixed_view = view(*is_fixed);
    is_fixed_view[0]   = 1;

    label_surface(mesh);
    label_triangle_orient(mesh);

    auto object = scene.objects().create("tets");
    object->geometries().create(mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    auto fem_system = world.features().find<FEMSystemFeature>();
    REQUIRE(fem_system);

    auto fem_state = world.features().find<FiniteElementStateAccessorFeature>();
    REQUIRE(fem_state);

    // uid discovery: SNH must be exportable
    auto uids = fem_system->fem_constitution_uids();
    REQUIRE(std::find(uids.begin(), uids.end(), snh.uid()) != uids.end());

    // unknown uid: warn and leave the geometry untouched, do not crash
    {
        Geometry unknown;
        fem_system->fem_energy(999999, unknown);
        CHECK(unknown.instances().find<Float>("energy") == nullptr);
    }

    // a geometry to read the backend state and perturb it
    auto state_geo = fem_state->create_geometry();
    state_geo.vertices().create<Vector3>(builtin::position, Vector3::Zero());
    fem_state->copy_to(state_geo);
    auto state_pos = view(*state_geo.vertices().find<Vector3>(builtin::position));

    vector<Vector3> positions(4);
    for(int i = 0; i < 4; ++i)
        positions[i] = state_pos[i];

    auto energy_at = [&](const vector<Vector3>& pos) -> Float
    {
        for(int i = 0; i < 4; ++i)
            state_pos[i] = pos[i];
        fem_state->copy_from(state_geo);

        Geometry energy_geo;
        fem_system->fem_energy(snh, energy_geo);
        return view(*energy_geo.instances().find<Float>("energy"))[0];
    };

    // 1) rest state: rigid-motion invariance, analytic SNK1 energy, topology
    {
        Geometry energy_geo;
        fem_system->fem_energy(snh, energy_geo);
        REQUIRE(energy_geo.instances().size() == 1);
        auto energy = view(*energy_geo.instances().find<Float>("energy"));
        const Float rest_energy = energy[0];

        REQUIRE(energy_geo.instances().find<Vector4i>("topo"));
        auto topo = view(*energy_geo.instances().find<Vector4i>("topo"));
        for(int i = 0; i < 4; ++i)
            CHECK(topo[0][i] == i);

        vector<Vector3> translated = positions;
        for(auto& x : translated)
            x += Vector3{0.5, -0.25, 1.0};
        CHECK(energy_at(translated) == Catch::Approx(rest_energy).epsilon(1e-9));

        Matrix3x3       R = AngleAxis{0.6, Vector3::UnitY()}.toRotationMatrix();
        vector<Vector3> rotated = positions;
        for(auto& x : rotated)
            x = R * x;
        CHECK(energy_at(rotated) == Catch::Approx(rest_energy).epsilon(1e-9));

        // SNK1 is nonzero at rest by design: Psi(I) = mu^2 / (2 * (lambda + mu))
        CHECK(rest_energy > 0);
        const Float rest_volume =
            std::abs(Ds(Vs[0], Vs[1], Vs[2], Vs[3]).determinant()) / 6.0;
        const Float E      = 1.0e4;
        const Float nu     = 0.3;
        const Float mu     = E / (2 * (1 + nu));
        const Float lambda = E * nu / ((1 + nu) * (1 - 2 * nu));
        const Float expected_rest_energy =
            mu * mu / (2 * (lambda + mu)) * rest_volume * dt * dt;
        CHECK(rest_energy == Catch::Approx(expected_rest_energy).epsilon(1e-9));

        // restore the reference state
        for(int i = 0; i < 4; ++i)
            state_pos[i] = positions[i];
        fem_state->copy_from(state_geo);
    }

    // let the tet deform for a few frames
    for(int i = 0; i < 10; ++i)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
    }
    fem_state->copy_to(state_geo);
    for(int i = 0; i < 4; ++i)
        positions[i] = state_pos[i];

    // 2) per-element energy: positive, recomputed at the current positions
    Float deformed_energy = energy_at(positions);
    CHECK(deformed_energy > 0);

    {
        vector<Vector3> perturbed = positions;
        perturbed[1] += Vector3{1e-3, 1e-3, 1e-3};
        CHECK(energy_at(perturbed) != deformed_energy);

        // restore the reference state
        for(int i = 0; i < 4; ++i)
            state_pos[i] = positions[i];
        fem_state->copy_from(state_geo);
    }

    // 3) per-element gradient matches the finite differences of the energy
    Geometry grad_geo;
    fem_system->fem_gradient(snh, grad_geo);
    REQUIRE(grad_geo.instances().size() == 4);

    vector<Vector3> gradient(4, Vector3::Zero());
    {
        auto idx  = view(*grad_geo.instances().find<IndexT>("i"));
        auto grad = view(*grad_geo.instances().find<Vector3>("grad"));
        for(int k = 0; k < 4; ++k)
            gradient[idx[k]] = grad[k];
    }

    const Float h = 1e-6;
    for(int v = 0; v < 4; ++v)
    {
        for(int d = 0; d < 3; ++d)
        {
            Float fd = fd_energy_derivative(energy_at, positions, v, d, h);
            CAPTURE(v, d, fd, gradient[v][d]);
            CHECK(fd == Catch::Approx(gradient[v][d]).epsilon(1e-4).margin(1e-6));
        }
    }

    // restore the state perturbed by the finite-difference probes
    for(int i = 0; i < 4; ++i)
        state_pos[i] = positions[i];
    fem_state->copy_from(state_geo);

    // 4) per-vertex aggregated gradient equals the sum of the doublets
    fem_system->fem_vertex_gradient(snh, state_geo);
    {
        auto vert_grad = view(*state_geo.vertices().find<Vector3>("gradient"));
        for(int v = 0; v < 4; ++v)
            CHECK(vert_grad[v].isApprox(gradient[v], 1e-9));
    }

    // 5) Hessian triplets match finite differences of the gradient
    auto gradient_at = [&](const vector<Vector3>& pos) -> vector<Vector3>
    {
        for(int i = 0; i < 4; ++i)
            state_pos[i] = pos[i];
        fem_state->copy_from(state_geo);

        Geometry this_grad_geo;
        fem_system->fem_gradient(snh, this_grad_geo);

        vector<Vector3> g(4, Vector3::Zero());
        auto idx  = view(*this_grad_geo.instances().find<IndexT>("i"));
        auto grad = view(*this_grad_geo.instances().find<Vector3>("grad"));
        for(int k = 0; k < 4; ++k)
            g[idx[k]] = grad[k];
        return g;
    };

    Geometry hess_geo;
    fem_system->fem_hessian(snh, hess_geo);
    REQUIRE(hess_geo.instances().size() == 10);

    auto hi    = view(*hess_geo.instances().find<IndexT>("i"));
    auto hj    = view(*hess_geo.instances().find<IndexT>("j"));
    auto hvals = view(*hess_geo.instances().find<Matrix3x3>("hess"));

    auto find_block = [&](IndexT i, IndexT j) -> const Matrix3x3&
    {
        for(int k = 0; k < 10; ++k)
            if(hi[k] == i && hj[k] == j)
                return hvals[k];
        FAIL(fmt::format("Hessian block ({},{}) not found", i, j));
        // unreachable
        return hvals[0];
    };

    // the diagonal must exist and be symmetric for every vertex
    for(int v = 0; v < 4; ++v)
    {
        const Matrix3x3& H = find_block(v, v);
        CHECK(H.isApprox(H.transpose(), 1e-9));
    }

    // spot-check (0,0), (0,1) and (1,3) against finite differences
    for(auto [i, j] : {std::pair{0, 0}, std::pair{0, 1}, std::pair{1, 3}})
    {
        const Matrix3x3& H = find_block(i, j);
        for(int s = 0; s < 3; ++s)
        {
            Vector3 fd_col = fd_gradient_derivative(gradient_at, positions, i, j, s, h);
            CAPTURE(i, j, s);
            CHECK(fd_col.isApprox(H.col(s), 1e-4));
        }
    }

    // restore the state perturbed by the finite-difference probes
    for(int i = 0; i < 4; ++i)
        state_pos[i] = positions[i];
    fem_state->copy_from(state_geo);
}

TEST_CASE("94_fem_data_export_spring", "[fem]")
{
    const vector<Vector3>  Vs = {Vector3{0, 0, 0}, Vector3{1, 0, 0}};
    const vector<Vector2i> Es = {Vector2i{0, 1}};

    const Float kappa     = 1.0e4;
    const Float thickness = 0.01_m;

    std::string this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                 = Scene::default_config();
    config["gravity"]           = Vector3{0, 0, 0};
    config["contact"]["enable"] = false;

    Scene scene{config};

    HookeanSpring spring;
    auto          mesh = linemesh(Vs, Es);
    spring.apply_to(mesh, kappa, 1e3, thickness);

    auto object = scene.objects().create("spring");
    object->geometries().create(mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    auto fem_system = world.features().find<FEMSystemFeature>();
    REQUIRE(fem_system);
    auto fem_state = world.features().find<FiniteElementStateAccessorFeature>();
    REQUIRE(fem_state);

    auto uids = fem_system->fem_constitution_uids();
    REQUIRE(std::find(uids.begin(), uids.end(), spring.uid()) != uids.end());

    auto state_geo = fem_state->create_geometry();
    state_geo.vertices().create<Vector3>(builtin::position, Vector3::Zero());
    fem_state->copy_to(state_geo);
    auto state_pos = view(*state_geo.vertices().find<Vector3>(builtin::position));

    vector<Vector3> positions(2);
    for(int i = 0; i < 2; ++i)
        positions[i] = state_pos[i];

    auto energy_at = [&](const vector<Vector3>& pos) -> Float
    {
        for(int i = 0; i < 2; ++i)
            state_pos[i] = pos[i];
        fem_state->copy_from(state_geo);

        Geometry energy_geo;
        fem_system->fem_energy(spring, energy_geo);
        return view(*energy_geo.instances().find<Float>("energy"))[0];
    };

    // 1) energy: zero at rest, analytic value when stretched
    CHECK(std::abs(energy_at(positions)) < 1e-10);

    const Float     dt        = 0.01;  // the default config dt
    vector<Vector3> stretched = positions;
    stretched[1][1] += 0.5;  // stretch along y: l = sqrt(1 + 0.25)
    const Float l  = (stretched[1] - stretched[0]).norm();
    const Float L0 = (positions[1] - positions[0]).norm();
    const Float expected_energy =
        0.5 * kappa * std::pow((l - L0) / L0, 2)
        * (L0 * std::numbers::pi * thickness * thickness) * dt * dt;

    Float exported = energy_at(stretched);
    CHECK(exported == Catch::Approx(expected_energy).epsilon(1e-6));

    {
        Geometry energy_geo;
        fem_system->fem_energy(spring, energy_geo);
        REQUIRE(energy_geo.instances().size() == 1);

        auto topo = energy_geo.instances().find<Vector2i>("topo");
        REQUIRE(topo);
        auto topo_view = view(*topo);
        CHECK(topo_view[0][0] == 0);
        CHECK(topo_view[0][1] == 1);
    }

    // 2) gradient vs finite differences, at the stretched state
    Geometry grad_geo;
    fem_system->fem_gradient(spring, grad_geo);
    REQUIRE(grad_geo.instances().size() == 2);

    vector<Vector3> gradient(2, Vector3::Zero());
    {
        auto idx  = view(*grad_geo.instances().find<IndexT>("i"));
        auto grad = view(*grad_geo.instances().find<Vector3>("grad"));
        for(int k = 0; k < 2; ++k)
            gradient[idx[k]] = grad[k];
    }

    const Float h = 1e-6;
    for(int v = 0; v < 2; ++v)
        for(int d = 0; d < 3; ++d)
        {
            Float fd = fd_energy_derivative(energy_at, stretched, v, d, h);
            CAPTURE(v, d, fd, gradient[v][d]);
            CHECK(fd == Catch::Approx(gradient[v][d]).epsilon(1e-4).margin(1e-6));
        }

    // restore the state perturbed by the finite-difference probes
    for(int i = 0; i < 2; ++i)
        state_pos[i] = stretched[i];
    fem_state->copy_from(state_geo);

    // 3) per-vertex aggregated gradient equals the sum of the doublets
    fem_system->fem_vertex_gradient(spring, state_geo);
    {
        auto vert_grad = view(*state_geo.vertices().find<Vector3>("gradient"));
        for(int v = 0; v < 2; ++v)
            CHECK(vert_grad[v].isApprox(gradient[v], 1e-9));
    }

    // 4) hessian: 3 upper-triangular blocks, symmetric, one FD spot-check
    Geometry hess_geo;
    fem_system->fem_hessian(spring, hess_geo);
    REQUIRE(hess_geo.instances().size() == 3);

    auto hi    = view(*hess_geo.instances().find<IndexT>("i"));
    auto hj    = view(*hess_geo.instances().find<IndexT>("j"));
    auto hvals = view(*hess_geo.instances().find<Matrix3x3>("hess"));

    auto find_block = [&](IndexT i, IndexT j) -> const Matrix3x3&
    {
        for(int k = 0; k < 3; ++k)
            if(hi[k] == i && hj[k] == j)
                return hvals[k];
        FAIL(fmt::format("Hessian block ({},{}) not found", i, j));
        return hvals[0];
    };

    for(int v = 0; v < 2; ++v)
    {
        const Matrix3x3& H = find_block(v, v);
        CHECK(H.isApprox(H.transpose(), 1e-9));
    }

    auto gradient_at = [&](const vector<Vector3>& pos) -> vector<Vector3>
    {
        for(int i = 0; i < 2; ++i)
            state_pos[i] = pos[i];
        fem_state->copy_from(state_geo);

        Geometry this_grad_geo;
        fem_system->fem_gradient(spring, this_grad_geo);

        vector<Vector3> g(2, Vector3::Zero());
        auto idx  = view(*this_grad_geo.instances().find<IndexT>("i"));
        auto grad = view(*this_grad_geo.instances().find<Vector3>("grad"));
        for(int k = 0; k < 2; ++k)
            g[idx[k]] = grad[k];
        return g;
    };

    {
        const Matrix3x3& H = find_block(0, 1);
        for(int s = 0; s < 3; ++s)
        {
            Vector3 fd_col = fd_gradient_derivative(gradient_at, stretched, 0, 1, s, h);
            CAPTURE(s);
            CHECK(fd_col.isApprox(H.col(s), 1e-4));
        }
    }

    // restore the rest state
    for(int i = 0; i < 2; ++i)
        state_pos[i] = positions[i];
    fem_state->copy_from(state_geo);
}
