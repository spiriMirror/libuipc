#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <finite_element/constitutions/plastic_discrete_shell_bending_function.h>
#include <limits>
#include <numbers>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <utils/dump_utils.h>
#include <kernel_cout.h>
#include <algorithm>
#include <cmath>

namespace uipc::backend::cuda
{
namespace
{
struct Vector2iHash
{
    size_t operator()(const Vector2i& v) const
    {
        size_t front = v[0];
        size_t end   = v[1];
        return front << 32 | end;
    }
};

struct StencilRecord
{
    Vector4i stencil;
    Float    bending_stiffness = 0.0;
    Float    yield_threshold   = 0.0;
    Float    hardening_modulus = 0.0;
};

bool stencil_less(const Vector4i& a, const Vector4i& b)
{
    for(int i = 0; i < 4; ++i)
    {
        if(a[i] != b[i])
            return a[i] < b[i];
    }

    return false;
}

bool stencil_equal(const Vector4i& a, const Vector4i& b)
{
    for(int i = 0; i < 4; ++i)
    {
        if(a[i] != b[i])
            return false;
    }

    return true;
}
}  // namespace

class PlasticDiscreteShellBending final : public FiniteElementExtraConstitution
{
    static constexpr U64   PlasticDiscreteShellBendingUID = 31;
    static constexpr SizeT StencilSize                    = 4;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;
    using Base = FiniteElementExtraConstitution;

  public:
    using Base::Base;
    U64 get_uid() const noexcept override { return PlasticDiscreteShellBendingUID; }

    class InitInfo
    {
      public:
        bool        valid_bending() const { return oppo_verts.size() == 2; }
        IndexT      edge_index = -1;
        set<IndexT> oppo_verts;
    };

    vector<Vector4i> h_stencils;
    vector<Float>    h_bending_stiffness;
    vector<Float>    h_rest_lengths;
    vector<Float>    h_h_bars;
    vector<Float>    h_theta_bars;
    vector<Float>    h_yield_thresholds;
    vector<Float>    h_hardening_moduli;
    vector<Float>    h_V_bars;

    muda::DeviceBuffer<Vector4i> stencils;
    muda::DeviceBuffer<Float>    bending_stiffnesses;
    muda::DeviceBuffer<Float>    rest_lengths;
    muda::DeviceBuffer<Float>    h_bars;
    muda::DeviceBuffer<Float>    theta_bars;
    muda::DeviceBuffer<Float>    yield_thresholds;
    muda::DeviceBuffer<Float>    hardening_moduli;
    muda::DeviceBuffer<Float>    V_bars;

    BufferDump dump_theta_bars;
    BufferDump dump_yield_thresholds;
    BufferDump dump_stencil_identities;

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_init(FilteredInfo& info) override
    {
        namespace PDSB = sym::plastic_discrete_shell_bending;

        using ForEachInfo = FiniteElementMethod::ForEachInfo;
        auto geo_slots    = world().scene().geometries();

        vector<StencilRecord> stencil_records;

        info.for_each(
            geo_slots,
            [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                unordered_map<Vector2i, InitInfo, Vector2iHash> stencil_map;

                auto vertex_offset =
                    sc.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
                UIPC_ASSERT(vertex_offset, "Vertex offset not found, why?");
                auto vertex_offset_v = vertex_offset->view().front();

                auto edges = sc.edges().topo().view();
                for(auto&& [i, e] : enumerate(edges))
                {
                    Vector2i E = e;
                    std::sort(E.begin(), E.end());
                    stencil_map[E].edge_index = i;
                }

                auto triangles = sc.triangles().topo().view();
                for(auto&& t : triangles)
                {
                    Vector3i T = t;
                    std::sort(T.begin(), T.end());

                    Vector2i E01 = {T[0], T[1]};
                    Vector2i E02 = {T[0], T[2]};
                    Vector2i E12 = {T[1], T[2]};

                    stencil_map[E01].oppo_verts.insert(T[2]);
                    stencil_map[E02].oppo_verts.insert(T[1]);
                    stencil_map[E12].oppo_verts.insert(T[0]);
                }

                auto bending_stiffnesses = sc.edges().find<Float>("bending_stiffness");
                auto yield_thresholds    = sc.edges().find<Float>("bending_yield_threshold");
                auto hardening_moduli    = sc.edges().find<Float>("bending_hardening_modulus");
                UIPC_ASSERT(bending_stiffnesses, "Bending stiffness not found, why?");
                UIPC_ASSERT(yield_thresholds, "Yield threshold not found, why?");
                UIPC_ASSERT(hardening_moduli, "Hardening modulus not found, why?");

                auto bs_view = bending_stiffnesses->view();
                auto yt_view = yield_thresholds->view();
                auto hm_view = hardening_moduli->view();

                for(auto&& [E, stencil_info] : stencil_map)
                {
                    if(!stencil_info.valid_bending())
                        continue;

                    Vector4i stencil{*stencil_info.oppo_verts.begin(),
                                     E(0),
                                     E(1),
                                     *stencil_info.oppo_verts.rbegin()};

                    stencil_records.push_back(StencilRecord{
                        .stencil           = stencil.array() + vertex_offset_v,
                        .bending_stiffness = bs_view[stencil_info.edge_index],
                        .yield_threshold   = yt_view[stencil_info.edge_index],
                        .hardening_modulus = hm_view[stencil_info.edge_index],
                    });
                }
            });

        std::ranges::sort(stencil_records,
                          [](const StencilRecord& a, const StencilRecord& b)
                          { return stencil_less(a.stencil, b.stencil); });

        h_stencils.resize(stencil_records.size());
        h_bending_stiffness.resize(stencil_records.size());
        h_yield_thresholds.resize(stencil_records.size());
        h_hardening_moduli.resize(stencil_records.size());
        for(auto&& [i, record] : enumerate(stencil_records))
        {
            h_stencils[i]            = record.stencil;
            h_bending_stiffness[i]   = record.bending_stiffness;
            h_yield_thresholds[i]    = record.yield_threshold;
            h_hardening_moduli[i]    = record.hardening_modulus;
        }

        auto x_bars      = info.rest_positions();
        auto thicknesses = info.thicknesses();
        h_rest_lengths.resize(h_stencils.size());
        h_h_bars.resize(h_stencils.size());
        h_theta_bars.resize(h_stencils.size());
        h_V_bars.resize(h_stencils.size());

        for(auto&& [i, stencil] : enumerate(h_stencils))
        {
            Vector3 X0         = x_bars[stencil[0]];
            Vector3 X1         = x_bars[stencil[1]];
            Vector3 X2         = x_bars[stencil[2]];
            Vector3 X3         = x_bars[stencil[3]];
            Float   thickness0 = thicknesses[stencil[0]];
            Float   thickness1 = thicknesses[stencil[1]];
            Float   thickness2 = thicknesses[stencil[2]];
            Float   thickness3 = thicknesses[stencil[3]];

            Float L0, V_bar, h_bar, theta_bar;
            PDSB::compute_constants(L0,
                                    h_bar,
                                    theta_bar,
                                    V_bar,
                                    X0,
                                    X1,
                                    X2,
                                    X3,
                                    thickness0,
                                    thickness1,
                                    thickness2,
                                    thickness3);

            h_rest_lengths[i] = L0;
            h_h_bars[i]       = h_bar;
            h_theta_bars[i]   = theta_bar;
            h_V_bars[i]       = V_bar;
        }

        stencils.resize(h_stencils.size());
        stencils.view().copy_from(h_stencils.data());

        bending_stiffnesses.resize(h_bending_stiffness.size());
        bending_stiffnesses.view().copy_from(h_bending_stiffness.data());

        rest_lengths.resize(h_rest_lengths.size());
        rest_lengths.view().copy_from(h_rest_lengths.data());

        h_bars.resize(h_h_bars.size());
        h_bars.view().copy_from(h_h_bars.data());

        theta_bars.resize(h_theta_bars.size());
        theta_bars.view().copy_from(h_theta_bars.data());

        yield_thresholds.resize(h_yield_thresholds.size());
        yield_thresholds.view().copy_from(h_yield_thresholds.data());

        hardening_moduli.resize(h_hardening_moduli.size());
        hardening_moduli.view().copy_from(h_hardening_moduli.data());

        V_bars.resize(h_V_bars.size());
        V_bars.view().copy_from(h_V_bars.data());
    }

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(stencils.size());
        info.gradient_count(stencils.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(stencils.size() * HalfHessianSize);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace PDSB = sym::plastic_discrete_shell_bending;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.energies().size(),
                   [stencils = stencils.viewer().name("stencils"),
                    bending_stiffnesses = bending_stiffnesses.viewer().name("bending_stiffness"),
                    theta_bars = theta_bars.viewer().name("theta_bar"),
                    h_bars     = h_bars.viewer().name("h_bar"),
                    V_bars     = V_bars.viewer().name("V_bar"),
                    L0s        = rest_lengths.viewer().name("rest_lengths"),
                    xs         = info.xs().viewer().name("xs"),
                    energies   = info.energies().viewer().name("energies"),
                    dt         = info.dt()] __device__(int I)
                   {
                       Vector4i stencil   = stencils(I);
                       Float    kappa     = bending_stiffnesses(I);
                       Float    L0        = L0s(I);
                       Float    h_bar     = h_bars(I);
                       Float    theta_bar = theta_bars(I);
                       Float    V_bar     = V_bars(I);

                       Vector3 x0 = xs(stencil[0]);
                       Vector3 x1 = xs(stencil[1]);
                       Vector3 x2 = xs(stencil[2]);
                       Vector3 x3 = xs(stencil[3]);

                       Float E = PDSB::E(x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       energies(I) = E * V_bar * dt * dt;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace PDSB = sym::plastic_discrete_shell_bending;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(stencils.size(),
                   [stencils = stencils.viewer().name("stencils"),
                    bending_stiffnesses = bending_stiffnesses.viewer().name("bending_stiffness"),
                    theta_bars = theta_bars.viewer().name("theta_bar"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    h_bars = h_bars.viewer().name("h_bar"),
                    V_bars = V_bars.viewer().name("V_bar"),
                    L0s    = rest_lengths.viewer().name("rest_lengths"),
                    xs     = info.xs().viewer().name("xs"),
                    G3s    = info.gradients().viewer().name("gradients"),
                    H3x3s  = info.hessians().viewer().name("hessians"),
                    dt     = info.dt(),
                    gradient_only = info.gradient_only()] __device__(int I) mutable
                   {
                       Vector4i stencil   = stencils(I);
                       Float    kappa     = bending_stiffnesses(I);
                       Float    L0        = L0s(I);
                       Float    h_bar     = h_bars(I);
                       Float    theta_bar = theta_bars(I);
                       Float    V_bar     = V_bars(I);

                       Vector3 x0 = xs(stencil[0]);
                       Vector3 x1 = xs(stencil[1]);
                       Vector3 x2 = xs(stencil[2]);
                       Vector3 x3 = xs(stencil[3]);

                       Float Vdt2 = V_bar * dt * dt;

                       Vector12    G12;
                       Matrix12x12 H12x12;

                       PDSB::dEdx(G12, x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       G12 *= Vdt2;
                       DoubletVectorAssembler DVA{G3s};
                       DVA.segment<StencilSize>(I * StencilSize).write(stencil, G12);

                       if(gradient_only)
                           return;

                       PDSB::ddEddx(H12x12, x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       H12x12 *= Vdt2;
                       make_spd(H12x12);

                       TripletMatrixAssembler TMA{H3x3s};
                       TMA.half_block<StencilSize>(I * HalfHessianSize).write(stencil, H12x12);
                   });
    }

    virtual void do_post_step(PostStepInfo& info) override
    {
        using namespace muda;
        namespace PDSB = sym::plastic_discrete_shell_bending;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(stencils.size(),
                   [stencils = stencils.viewer().name("stencils"),
                    theta_bars = theta_bars.viewer().name("theta_bar"),
                    yield_thresholds = yield_thresholds.viewer().name("yield_threshold"),
                    hardening_moduli = hardening_moduli.viewer().name("hardening_modulus"),
                    xs = info.xs().viewer().name("xs")] __device__(int I) mutable
                   {
                       Vector4i stencil = stencils(I);

                       Vector3 x0 = xs(stencil[0]);
                       Vector3 x1 = xs(stencil[1]);
                       Vector3 x2 = xs(stencil[2]);
                       Vector3 x3 = xs(stencil[3]);

                       Float theta = 0.0;
                       if(!PDSB::safe_dihedral_angle(x0, x1, x2, x3, theta))
                           return;

                       Float theta_bar       = theta_bars(I);
                       Float yield_threshold = yield_thresholds(I);
                       Float hardening       = hardening_moduli(I);

                       if(PDSB::update_plastic_state<Float>(
                              theta, theta_bar, yield_threshold, hardening))
                       {
                           theta_bars(I)       = theta_bar;
                           yield_thresholds(I) = yield_threshold;
                       }
                   });
    }

    virtual bool do_dump(DumpInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        return dump_theta_bars.dump(fmt::format("{}theta_bar.{}", path, frame), theta_bars)
               && dump_yield_thresholds.dump(
                    fmt::format("{}yield_threshold.{}", path, frame), yield_thresholds)
               && dump_stencil_identities.dump(
                   fmt::format("{}stencil_identity.{}", path, frame),
                   span<const Vector4i>{h_stencils.data(), h_stencils.size()});
    }

    virtual bool do_try_recover(RecoverInfo& info) override
    {
        auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
        auto frame = info.frame();

        auto cleanup_recover = [&]
        {
            dump_theta_bars.clean_up();
            dump_yield_thresholds.clean_up();
            dump_stencil_identities.clean_up();
        };

        const bool loaded = dump_theta_bars.load(fmt::format("{}theta_bar.{}", path, frame))
                            && dump_yield_thresholds.load(
                                fmt::format("{}yield_threshold.{}", path, frame))
                            && dump_stencil_identities.load(
                                fmt::format("{}stencil_identity.{}", path, frame));
        if(!loaded)
        {
            cleanup_recover();
            return false;
        }

        auto theta_view = dump_theta_bars.view<Float>();
        auto yield_view = dump_yield_thresholds.view<Float>();
        auto stencil_identity_view = dump_stencil_identities.view<Vector4i>();

        const auto expected_size = stencils.size();
        if(theta_view.size() != expected_size || yield_view.size() != expected_size
           || stencil_identity_view.size() != expected_size)
        {
            logger::warn("PlasticDiscreteShellBending recover size mismatch: theta={} yield={} identity={} expected={}",
                         theta_view.size(),
                         yield_view.size(),
                         stencil_identity_view.size(),
                         expected_size);
            cleanup_recover();
            return false;
        }

        const auto invalid_theta = std::ranges::find_if(theta_view,
                                                        [](Float v) { return !std::isfinite(v); });
        const auto invalid_yield = std::ranges::find_if(
            yield_view, [](Float v) { return !std::isfinite(v) || v < 0.0; });

        if(invalid_theta != theta_view.end() || invalid_yield != yield_view.end())
        {
            logger::warn("PlasticDiscreteShellBending recover rejected non-finite plastic state");
            cleanup_recover();
            return false;
        }

        for(SizeT i = 0; i < expected_size; ++i)
        {
            if(!stencil_equal(stencil_identity_view[i], h_stencils[i]))
            {
                logger::warn("PlasticDiscreteShellBending recover rejected stencil identity mismatch at index {}",
                             i);
                cleanup_recover();
                return false;
            }
        }

        return true;
    }

    virtual void do_apply_recover(RecoverInfo& info) override
    {
        dump_theta_bars.apply_to(theta_bars);
        dump_yield_thresholds.apply_to(yield_thresholds);
    }

    virtual void do_clear_recover(RecoverInfo& info) override
    {
        dump_theta_bars.clean_up();
        dump_yield_thresholds.clean_up();
        dump_stencil_identities.clean_up();
    }
};

REGISTER_SIM_SYSTEM(PlasticDiscreteShellBending);
}  // namespace uipc::backend::cuda
