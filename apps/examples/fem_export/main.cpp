#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/core/fem_system_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <uipc/core/contact_system_feature.h>
#include <uipc/builtin/attribute_name.h>
#include <fmt/format.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>

using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

/**
 * @brief Energy-conservation testbench: a StableNeoHookean tetrahedron
 *        dropped onto an implicit ground plane under gravity.
 *
 * Per frame, the total elastic energy, the kinetic energy, the
 * gravitational potential energy and the contact (IPC barrier) energy
 * are written to `fem_energy.csv`. Both energy exports (FEMSystemFeature,
 * ContactSystemFeature) carry the solver's dt^2 incremental-potential
 * scaling and are divided by dt^2 here. SNK1 is nonzero at the rest state
 * by design; since the tet is in free fall on the first frame, its
 * elastic energy there is exactly that constant rest offset, which is
 * subtracted from the following frames.
 *
 * Expected behavior:
 * - free fall: a small constant drain of 1/2 m g^2 dt^2 per step, the
 *   BDF1 (backward Euler) discretization dissipation; the elastic column
 *   stays at ~0;
 * - impact: a larger drop, the numerical contact dissipation of the
 *   non-symplectic integrator (the barrier itself stores only ~J at
 *   rest here);
 * - rest: the tet stays squashed on the plane under its own weight.
 *
 * Read the CSV with e.g. pandas/matplotlib:
 *   df = pandas.read_csv("fem_energy.csv")
 *   df.plot(x="time", y=["elastic_energy", "kinetic_energy",
 *                        "potential_energy", "total_energy"])
 *
 * Ref: https://github.com/spiriMirror/libuipc/issues/344
 */
int main()
{
    Engine engine{"cuda"};
    logger::set_level(spdlog::level::warn);

    World world{engine};

    const Float dt = 0.01_s;

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["dt"]                            = dt;
    config["contact"]["friction"]["enable"] = 0;  // frictionless bounce

    Scene scene{config};

    StableNeoHookean snh_constitution;

    // a regular tetrahedron, lifted above the ground
    Vector3          lift{0, 0.5, 0};
    vector<Vector3>  Vs = {Vector3{0, 0, 1} + lift,
                           Vector3{0, -1, 0} + lift,
                           Vector3{-std::sqrt(3) / 2, 0, -0.5} + lift,
                           Vector3{std::sqrt(3) / 2, 0, -0.5} + lift};
    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    SimplicialComplex tet = tetmesh(Vs, Ts);

    const Float density = 1e3;
    snh_constitution.apply_to(tet, ElasticModuli::youngs_poisson(1.0e4, 0.3), density);

    // frictionless contact with the implicit ground
    auto default_contact = scene.contact_tabular().default_element();
    scene.contact_tabular().default_model(0.0, 1.0_GPa);
    default_contact.apply_to(tet);

    label_surface(tet);
    label_triangle_orient(tet);

    // per-vertex masses, matching the backend mass assembly:
    // mass = mass_density * lumped vertex volume (tet volume / 4)
    vector<Float> vertex_mass(4);
    {
        auto volume = tet.vertices().find<Float>(builtin::volume);
        UIPC_ASSERT(volume, "`builtin::volume` missing: it is created by `apply_to`");
        auto volume_view = view(*volume);
        std::ranges::transform(volume_view,
                               vertex_mass.begin(),
                               [&](Float v) { return density * v; });
    }

    auto object = scene.objects().create("tet");
    object->geometries().create(tet);

    // implicit ground plane at y = -1
    auto ground_object = scene.objects().create("ground");
    ground_object->geometries().create(ground(-1.0, Vector3::UnitY()));

    world.init(scene);

    auto this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    std::ofstream csv{this_output_path + "fem_energy.csv"};
    UIPC_ASSERT(csv.is_open(), "Failed to open fem_energy.csv for writing.");
    csv << "frame,"
        << "time,"
        << "elastic_energy,"
        << "kinetic_energy,"
        << "potential_energy,"
        << "contact_energy,"
        << "total_energy"
        << "\n";

    auto fem_system = world.features().find<FEMSystemFeature>();
    UIPC_ASSERT_THROW(fem_system, "FEMSystemFeature not found: this example requires the CUDA backend.");

    auto fem_state = world.features().find<FiniteElementStateAccessorFeature>();
    UIPC_ASSERT_THROW(fem_state, "FiniteElementStateAccessorFeature not found.");

    auto contact_system = world.features().find<ContactSystemFeature>();
    UIPC_ASSERT_THROW(contact_system, "ContactSystemFeature not found.");
    auto contact_types = contact_system->contact_primitive_types();

    auto state_geo = fem_state->create_geometry();
    state_geo.vertices().create<Vector3>(builtin::position, Vector3::Zero());
    state_geo.vertices().create<Vector3>(builtin::velocity, Vector3::Zero());

    Float    export_seconds = 0.0;
    uint64_t export_calls   = 0;

    // SNK1's constant rest-energy offset (nonzero at rest by design). The tet
    // is in free fall on the first frame, so its elastic energy there is
    // exactly the offset; subtract it from the following frames.
    Float elastic_offset = 0.0;

    const int total_frames = 300;
    for(int frame = 1; frame <= total_frames; ++frame)
    {
        world.advance();
        world.sync();
        world.retrieve();

        fem_state->copy_to(state_geo);
        auto positions = view(*state_geo.vertices().find<Vector3>(builtin::position));
        auto velocities = view(*state_geo.vertices().find<Vector3>(builtin::velocity));

        // per-element elastic energy, dt^2-scaled like the solver's
        // incremental potential
        Geometry energy_geo;
        auto     t0 = std::chrono::steady_clock::now();
        fem_system->fem_energy(snh_constitution, energy_geo);
        auto t1 = std::chrono::steady_clock::now();
        export_seconds += std::chrono::duration<Float>(t1 - t0).count();
        ++export_calls;

        auto energies = view(*energy_geo.instances().find<Float>("energy"));
        const Float exported_elastic =
            std::accumulate(energies.begin(), energies.end(), 0.0);
        if(frame == 1)
            elastic_offset = exported_elastic;
        const Float elastic_energy = (exported_elastic - elastic_offset) / (dt * dt);

        Float kinetic_energy   = 0.0;  // 1/2 m v^2
        Float potential_energy = 0.0;  // m g h, datum at y = 0
        for(int i = 0; i < 4; ++i)
        {
            kinetic_energy += 0.5 * vertex_mass[i] * velocities[i].squaredNorm();
            potential_energy += vertex_mass[i] * 9.8 * positions[i].y();
        }

        // contact (IPC barrier) energy of the contact pairs, via
        // ContactSystemFeature. The exported values are the solver's
        // dt^2-scaled incremental-potential energies, so divide by dt^2 to
        // make them comparable with the physical energies above.
        Float contact_energy = 0.0;
        for(auto&& prim_type : contact_types)
        {
            Geometry contact_geo;
            contact_system->contact_energy(prim_type, contact_geo);
            if(auto e = contact_geo.instances().find<Float>("energy"))
            {
                auto ev = view(*e);
                contact_energy += std::accumulate(ev.begin(), ev.end(), 0.0);
            }
        }
        contact_energy /= dt * dt;

        const Float total_energy =
            elastic_energy + kinetic_energy + potential_energy + contact_energy;

        csv << fmt::format("{}, {}, {}, {}, {}, {}, {}\n",
                           frame,
                           frame * dt,
                           elastic_energy,
                           kinetic_energy,
                           potential_energy,
                           contact_energy,
                           total_energy);

        if(frame % 50 == 0)
        {
            fmt::print(
                "[{:3d}] elastic: {:>11.6e} J | kinetic: {:>11.6e} J | "
                "potential: {:>11.6e} J | contact: {:>11.6e} | total: {:>11.6e} J\n",
                frame,
                elastic_energy,
                kinetic_energy,
                potential_energy,
                contact_energy,
                total_energy);
        }
    }

    csv.close();
    fmt::print("Wrote {}\n", this_output_path + "fem_energy.csv");
    fmt::print("Average fem_energy export time: {:.3f} ms/call\n",
               export_seconds / export_calls * 1000.0);
}
