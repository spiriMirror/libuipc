#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/plastic_discrete_shell_bending.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
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

S<geometry::GeometrySlot> build_scene(Scene& scene)
{
    NeoHookeanShell             nhs;
    PlasticDiscreteShellBending pdsb;
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

std::filesystem::path find_dump_file(const std::filesystem::path& dump_root,
                                     std::string_view             stem)
{
    for(const auto& entry : std::filesystem::recursive_directory_iterator(dump_root))
    {
        if(entry.is_regular_file() && entry.path().filename() == stem)
            return entry.path();
    }

    return {};
}

template <typename T>
void write_dump(const std::filesystem::path& path, span<const T> values)
{
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    REQUIRE(ofs.is_open());

    std::size_t size_bytes = values.size() * sizeof(T);
    ofs.write(reinterpret_cast<const char*>(&DumpMagic), sizeof(DumpMagic));
    ofs.write(reinterpret_cast<const char*>(&size_bytes), sizeof(size_bytes));
    ofs.write(reinterpret_cast<const char*>(values.data()), size_bytes);
}

template <typename T>
vector<T> read_dump(const std::filesystem::path& path)
{
    std::ifstream ifs(path, std::ios::binary);
    REQUIRE(ifs.is_open());

    std::size_t magic      = 0;
    std::size_t size_bytes = 0;
    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&size_bytes), sizeof(size_bytes));

    REQUIRE(magic == DumpMagic);
    REQUIRE(size_bytes % sizeof(T) == 0);

    vector<T> values(size_bytes / sizeof(T));
    if(!values.empty())
        ifs.read(reinterpret_cast<char*>(values.data()), size_bytes);
    return values;
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

    SimulationResult result;
    if(recover_frame)
    {
        result.recover_result = world.recover(*recover_frame);
        if(!result.recover_result)
            return result;

        REQUIRE(world.frame() == *recover_frame);
    }

    while(world.frame() < EndFrame)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
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

TEST_CASE("73_plastic_discrete_shell_bending_recover", "[fem][plastic_dsb][recover]")
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

    SECTION("recover_rejects_size_mismatch_dump")
    {
        auto baseline = run_case(workspace, true);
        (void)baseline;

        auto dump_root   = workspace / "dump";
        auto yield_file  = find_dump_file(dump_root, fmt::format("yield_threshold.{}", RecoverFrame));
        REQUIRE(!yield_file.empty());

        const vector<Float> wrong_size = {0.25, 0.5};
        write_dump(yield_file, span<const Float>{wrong_size.data(), wrong_size.size()});

        auto resumed = run_case(workspace, false, RecoverFrame);
        CHECK_FALSE(resumed.recover_result);
    }

    SECTION("recover_rejects_non_finite_dump")
    {
        auto baseline = run_case(workspace, true);
        (void)baseline;

        auto dump_root  = workspace / "dump";
        auto theta_file = find_dump_file(dump_root, fmt::format("theta_bar.{}", RecoverFrame));
        REQUIRE(!theta_file.empty());

        auto invalid_values = read_dump<Float>(theta_file);
        REQUIRE(!invalid_values.empty());
        invalid_values[0] = std::numeric_limits<Float>::quiet_NaN();
        write_dump(theta_file, span<const Float>{invalid_values.data(), invalid_values.size()});

        auto resumed = run_case(workspace, false, RecoverFrame);
        CHECK_FALSE(resumed.recover_result);
    }

    SECTION("recover_rejects_stencil_identity_mismatch")
    {
        auto baseline = run_case(workspace, true);
        (void)baseline;

        auto dump_root      = workspace / "dump";
        auto stencil_file   = find_dump_file(dump_root,
                                           fmt::format("stencil_identity.{}", RecoverFrame));
        REQUIRE(!stencil_file.empty());

        auto wrong_identity = read_dump<Vector4i>(stencil_file);
        REQUIRE(!wrong_identity.empty());
        auto swapped = wrong_identity[0];
        std::swap(swapped(1), swapped(2));
        wrong_identity[0] = swapped;
        write_dump(stencil_file,
                   span<const Vector4i>{wrong_identity.data(), wrong_identity.size()});

        auto resumed = run_case(workspace, false, RecoverFrame);
        CHECK_FALSE(resumed.recover_result);
    }
}
