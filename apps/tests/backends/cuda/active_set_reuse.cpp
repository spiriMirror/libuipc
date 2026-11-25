/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <uipc/core/contact_system_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <cstdlib>

namespace
{
void set_env(const char* name, const char* value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}
}  // namespace

// The IPC pipeline starts a frame from the previous frame's active set while
// the vertex positions are those of its last line-search trial. With
// UIPC_CHECK_ACTIVE_SET_REUSE=1 every such frame still runs the DCD pass and
// fails unless the reused and the DCD PT/EE/PE/PP/PH multisets are equal.
// Position overwrites through the state accessors and a recover must make the
// next frame run the DCD pass.
TEST_CASE("frame-start active-set reuse", "[cuda][collision detection]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    set_env("UIPC_CHECK_ACTIVE_SET_REUSE", "1");
    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        StableNeoHookean       snh;
        AffineBodyConstitution abd;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);

        SimplicialComplexIO io;
        auto cube = io.read(fmt::format("{}cube.msh", tetmesh_dir));
        label_surface(cube);
        label_triangle_orient(cube);

        // An affine-body cube on the ground and an FEM cube on it, both
        // within d_hat (0.01) of their supports from the first frame on.
        SimplicialComplex lower = cube;
        abd.apply_to(lower, 1.0_MPa);
        SimplicialComplex upper = cube;
        for(auto& p : view(upper.positions()))
            p += Vector3{0.3, 1.005, 0.2};
        snh.apply_to(upper, ElasticModuli::youngs_poisson(20.0_kPa, 0.49));

        auto object = scene.objects().create("cubes");
        object->geometries().create(lower);
        object->geometries().create(upper);
        object->geometries().create(ground(-0.505));
    }
    world.init(scene);
    set_env("UIPC_CHECK_ACTIVE_SET_REUSE", "0");
    REQUIRE(world.is_valid());

    auto abd_accessor = world.features().find<AffineBodyStateAccessorFeature>();
    auto fem_accessor = world.features().find<FiniteElementStateAccessorFeature>();
    auto contact = world.features().find<ContactSystemFeature>();
    REQUIRE(abd_accessor);
    REQUIRE(fem_accessor);
    REQUIRE(contact);

    SimplicialComplex abd_state = abd_accessor->create_geometry();
    abd_state.instances().create<Matrix4x4>(builtin::transform);
    SimplicialComplex fem_state = fem_accessor->create_geometry();
    fem_state.vertices().create<Vector3>(builtin::position);

    SizeT simplex_contacts = 0;
    auto  advance          = [&](SizeT frames, bool first_reuses)
    {
        for(SizeT i = 0; i < frames; ++i)
        {
            world.advance();
            REQUIRE(world.is_valid());
            world.retrieve();

            auto stats = engine.frame_stats();
            CAPTURE(world.frame());
            REQUIRE(stats["completed"].get<bool>());
            REQUIRE(stats["active_set_reused"].get<bool>() == (i > 0 || first_reuses));

            for(auto prim_type : {"PT+N", "EE+N", "PE+N", "PP+N"})
            {
                Geometry energy;
                contact->contact_energy(prim_type, energy);
                simplex_contacts += energy.instances().size();
            }
        }
    };

    advance(10, false);  // frames 1-10: the first frame has nothing to reuse

    // Lift the FEM cube by 5 d_hat.
    fem_accessor->copy_to(fem_state);
    for(auto& p : view(fem_state.positions()))
        p.y() += 0.05;
    fem_accessor->copy_from(fem_state);
    advance(10, false);  // frames 11-20

    // Shift the affine-body cube.
    abd_accessor->copy_to(abd_state);
    view(abd_state.transforms())[0](0, 3) += 0.2;
    abd_accessor->copy_from(abd_state);
    advance(5, false);  // frames 21-25

    REQUIRE(world.dump());
    advance(3, true);  // frames 26-28
    REQUIRE(world.recover(25));
    REQUIRE(world.frame() == 25);
    advance(5, false);  // frames 26-30 again

    REQUIRE(simplex_contacts > 0);
}
