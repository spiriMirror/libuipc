/**
 * Diagnostic test: compare MAS vs Diag linear solve quality at tol_rate=1e-12.
 *
 * Strategy (from user):
 *  1. Find the divergence frame.
 *  2. Run Diag and MAS with the same DOF state (synced via AffineBodyStateAccessorFeature).
 *  3. Use linear_pcg + dump_linear_pcg to dump b, r, z at the first Newton step.
 *  4. Compare PCG iteration counts: MAS should need FEWER iterations than Diag.
 *  5. Use Python to load dumps and verify with scipy sparse solve.
 *
 * With tol_rate=1e-12, both PCG runs solve the SAME Ax=b to full precision.
 * If MAS needs MORE iterations, the preconditioner is wrong.
 * If the b vectors differ between Diag and MAS, the Hessian assembly is polluted.
 */

#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/core/affine_body_state_accessor_feature.h>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

constexpr int   N_FRAMES   = 5;    // run 5 frames to find divergence frame
constexpr int   CHAIN_LEN  = 100;
constexpr Float STIFFNESS  = 100.0_MPa;
constexpr Float spacing    = 0.30;

// --- helpers -----------------------------------------------------------------

static void setup_scene(Scene& scene)
{
    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};

    AffineBodyConstitution       abd;
    AffineBodyRevoluteJoint      revolute_joint;

    vector<S<SimplicialComplexSlot>> body_slots(CHAIN_LEN);

    for(int i = 0; i < CHAIN_LEN; i++)
    {
        auto obj  = scene.objects().create(fmt::format("body_{}", i));
        auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        abd.apply_to(mesh, STIFFNESS);
        label_surface(mesh);

        Transform t = Transform::Identity();
        t.translate(Vector3{i * spacing, 0, 0});
        view(mesh.transforms())[0] = t.matrix();

        if(i == 0)
        {
            auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;
        }

        auto [slot, _] = obj->geometries().create(mesh);
        body_slots[i]  = slot;
    }

    vector<Vector3>                  l_pos0, l_pos1, r_pos0, r_pos1;
    vector<S<SimplicialComplexSlot>> l_slots, r_slots;
    vector<IndexT>                   l_ids, r_ids;
    vector<Float>                    strengths;

    for(int i = 0; i < CHAIN_LEN - 1; i++)
    {
        l_pos0.push_back(Vector3{ 0.15, 0.0, -0.15});
        l_pos1.push_back(Vector3{ 0.15, 0.0,  0.15});
        r_pos0.push_back(Vector3{-0.15, 0.0, -0.15});
        r_pos1.push_back(Vector3{-0.15, 0.0,  0.15});
        l_slots.push_back(body_slots[i]);
        r_slots.push_back(body_slots[i + 1]);
        l_ids.push_back(0);
        r_ids.push_back(0);
        strengths.push_back(100.0);
    }

    auto joint_mesh = revolute_joint.create_geometry(span{l_pos0},
                                                      span{l_pos1},
                                                      span{r_pos0},
                                                      span{r_pos1},
                                                      span{l_slots},
                                                      span{l_ids},
                                                      span{r_slots},
                                                      span{r_ids},
                                                      span{strengths});

    auto joint_obj = scene.objects().create("revolute_joints");
    joint_obj->geometries().create(joint_mesh);
}

static Json make_config(bool use_mas, bool do_dump)
{
    auto config = test::Scene::default_config();
    config["gravity"]                        = Vector3{0, -9.8, 0};
    config["contact"]["enable"]              = false;
    // Use non-fused PCG that supports vector dumps
    config["linear_system"]["solver"]        = std::string{"linear_pcg"};
    // Tight tolerance: both Diag and MAS must converge to the true solution.
    // If MAS needs MORE iterations than Diag at this tol, MAS preconditioner degrades convergence.
    config["linear_system"]["tol_rate"]      = 1e-12;
    config["dt"]                             = 0.01;

    if(!use_mas)
        config["extras"]["precond"]["force_abd_diag"] = 1;
    else
    {
        // Use fine-level only (block Jacobi): avoids over-correction from undamped
        // coarse levels on non-overlapping 1D chain topology.  Set to 0 to test full MAS.
        config["extras"]["precond"]["abd_mas_active_levels"] = 1;
    }

    // Enable dumps (b, r, z at PCG iter 0 + p, Ap at iter 1)
    if(do_dump)
    {
        config["extras"]["debug"]["dump_linear_pcg"]    = 1;
        config["extras"]["debug"]["dump_linear_system"] = 1;
    }

    return config;
}
}  // namespace


TEST_CASE("94_abd_mas_linear_solve_debug", "[abd][mas][debug]")
{
    using namespace uipc;

    auto base_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    auto diag_path = std::string(base_path) + "diag/";
    auto mas_path  = std::string(base_path) + "mas/";
    std::filesystem::create_directories(diag_path);
    std::filesystem::create_directories(mas_path);

    // -------------------------------------------------------------------------
    // Step 1: Run DIAG for N_FRAMES. Capture DOF state after each frame so
    //         MAS can be seeded with the SAME physical state at each frame.
    // -------------------------------------------------------------------------
    vector<geometry::SimplicialComplex> diag_dof_snapshots; // DOF state BEFORE each advance()
    vector<int>                         diag_pcg_iters;     // collected from log

    {
        auto config = make_config(false, true);  // Diag + dump
        Engine engine{"cuda", diag_path};
        World  world{engine};
        Scene  scene{config};
        setup_scene(scene);
        world.init(scene);

        auto abd_acc = world.features().find<AffineBodyStateAccessorFeature>();
        REQUIRE(abd_acc);

        // Build the geometry template that holds transforms + velocities
        auto state_geo = abd_acc->create_geometry();
        // Add needed attributes
        state_geo.instances().create<Matrix4x4>(builtin::transform);
        // velocity attribute name varies; create it if needed
        // (copy_to will skip attributes that don't exist in the target)

        logger::info("=== DIAG run for {} frames ===", N_FRAMES);
        // Snapshot state BEFORE advancing (frame 0 = initial state)
        abd_acc->copy_to(state_geo);
        diag_dof_snapshots.push_back(state_geo);

        for(int f = 0; f < N_FRAMES; f++)
        {
            world.advance();
            world.retrieve();

            // Snapshot state AFTER advancing (= BEFORE next advance)
            abd_acc->copy_to(state_geo);
            diag_dof_snapshots.push_back(state_geo);

            logger::info("[DIAG] frame {} done. valid={}", world.frame(), world.is_valid());
            if(!world.is_valid())
            {
                logger::warn("[DIAG] DIVERGED at frame {}", world.frame());
                break;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Step 2: Run MAS for N_FRAMES, SEEDING each frame from the DIAG snapshot.
    //         This ensures both simulations always solve the same Ax=b system
    //         (same Hessian + same gradient) at each Newton step.
    //         => PCG iteration count difference is PURELY the preconditioner.
    // -------------------------------------------------------------------------
    {
        auto config = make_config(true, true);  // MAS + dump
        Engine engine{"cuda", mas_path};
        World  world{engine};
        Scene  scene{config};
        setup_scene(scene);
        world.init(scene);

        auto abd_acc = world.features().find<AffineBodyStateAccessorFeature>();
        REQUIRE(abd_acc);

        logger::info("=== MAS run for {} frames (DOF synced from DIAG) ===", N_FRAMES);

        for(int f = 0; f < N_FRAMES && f < (int)diag_dof_snapshots.size() - 1; f++)
        {
            // Sync DOF to Diag's state BEFORE this frame's advance()
            abd_acc->copy_from(diag_dof_snapshots[f]);

            world.advance();
            world.retrieve();

            logger::info("[MAS] frame {} done. valid={}", world.frame(), world.is_valid());
            if(!world.is_valid())
            {
                logger::warn("[MAS] DIVERGED at frame {}", world.frame());
                break;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Step 3: Point user to the dumps for Python analysis.
    // -------------------------------------------------------------------------
    logger::info("");
    logger::info("=== Dump files for Python analysis ===");
    logger::info("  Diag dumps : {}debug/", diag_path);
    logger::info("  MAS  dumps : {}debug/", mas_path);
    logger::info("");
    logger::info("Run: python scripts/compare_linear_solve.py \"{0}\" \"{1}\"",
                 diag_path, mas_path);
    logger::info("");
    logger::info("Key files at frame=1, newton=0:");
    logger::info("  b.1.0.mtx   - gradient (MUST match between Diag and MAS)");
    logger::info("  r.1.0.0.mtx - residual at iter 0 (= b, MUST match)");
    logger::info("  z.1.0.0.mtx - precond response (will differ; MAS should be better)");
    logger::info("  p.1.mtx     - first search dir  (= z.1.0.0 for 1st iter)");
    logger::info("  Ap.1.mtx    - A*p               (MUST match between Diag and MAS)");
    logger::info("  => If b_diag != b_mas: Hessian/gradient assembly is different!");
    logger::info("  => If Ap_diag != Ap_mas: SpMV or Hessian is polluted!");
    logger::info("  => If z_mas is in wrong direction (dot(z_mas,r)<0): MAS is broken!");
}
