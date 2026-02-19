#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <finite_element/constitutions/discrete_shell_bending_function.h>
#include <numbers>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <kernel_cout.h>

namespace std
{
// hash function for Vector2i
template <>
struct hash<uipc::Vector2i>
{
    size_t operator()(const uipc::Vector2i& v) const
    {
        size_t front = v[0];
        size_t end   = v[1];
        return front << 32 | end;
    }
};
}  // namespace std

namespace uipc::backend::cuda
{
class DiscreteShellBending final : public FiniteElementExtraConstitution
{
    static constexpr U64   DiscreteShellBendingUID = 17;
    static constexpr SizeT StencilSize             = 4;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;
    using Base = FiniteElementExtraConstitution;

  public:
    using Base::Base;
    U64 get_uid() const noexcept override { return DiscreteShellBendingUID; }

    class InitInfo
    {
      public:
        bool        valid_bending() const { return oppo_verts.size() == 2; }
        IndexT      edge_index = -1;
        set<IndexT> oppo_verts;
        Float       stiffness = 0.0;
    };

    vector<Vector4i> h_stencils;  // X0, X1, X2, X3; (X1, X2) is middle edge
    vector<Float>    h_bending_stiffness;
    vector<Float>    h_rest_volumes;
    vector<Float>    h_rest_lengths;
    vector<Float>    h_h_bars;
    vector<Float>    h_theta_bars;
    vector<Float>    h_V_bars;

    muda::DeviceBuffer<Vector4i> stencils;  // X0, X1, X2, X3; (X1, X2) is middle edge
    muda::DeviceBuffer<Float> bending_stiffnesses;
    muda::DeviceBuffer<Float> rest_lengths;
    muda::DeviceBuffer<Float> h_bars;
    muda::DeviceBuffer<Float> theta_bars;
    muda::DeviceBuffer<Float> V_bars;

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_init(FilteredInfo& info) override
    {
        namespace DSB = sym::discrete_shell_bending;

        using ForEachInfo = FiniteElementMethod::ForEachInfo;
        auto geo_slots    = world().scene().geometries();

        list<Vector4i> stencil_list;
        list<Float>    bending_stiffness_list;

        // 1) Retrieve Quad Stencils
        info.for_each(  //
            geo_slots,
            [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                unordered_map<Vector2i, InitInfo> stencil_map;  // Edge -> opposite vertices

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

                    // insert opposite vertices
                    stencil_map[E01].oppo_verts.insert(T[2]);
                    stencil_map[E02].oppo_verts.insert(T[1]);
                    stencil_map[E12].oppo_verts.insert(T[0]);
                }

                auto bending_stiffnesses = sc.edges().find<Float>("bending_stiffness");
                UIPC_ASSERT(bending_stiffnesses, "Bending stiffness not found, why?");
                auto bs_view = bending_stiffnesses->view();

                for(auto&& [E, info] : stencil_map)
                {
                    if(info.valid_bending())
                    {
                        // X0, X1, X2, X3; (X1, X2) is middle edge
                        Vector4i stencil{*info.oppo_verts.begin(),    // X0
                                         E(0),                        // X1
                                         E(1),                        // X2
                                         *info.oppo_verts.rbegin()};  // X3

                        // convert to fem vertex index
                        stencil_list.push_back(stencil.array() + vertex_offset_v);

                        Float bs = bs_view[info.edge_index];
                        bending_stiffness_list.push_back(bs);
                    }
                }
            });

        // 2) Setup Invariant Data
        h_stencils.resize(stencil_list.size());
        h_bending_stiffness.resize(stencil_list.size());
        std::ranges::move(stencil_list, h_stencils.begin());
        std::ranges::move(bending_stiffness_list, h_bending_stiffness.begin());

        // 3) Setup Related Data
        span x_bars      = info.rest_positions();
        span thicknesses = info.thicknesses();
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
            DSB::compute_constants(L0,
                                   h_bar,
                                   theta_bar,
                                   V_bar,  //
                                   X0,
                                   X1,
                                   X2,
                                   X3,  //
                                   thickness0,
                                   thickness1,
                                   thickness2,
                                   thickness3);


            h_rest_lengths[i] = L0;
            h_h_bars[i]       = h_bar;
            h_theta_bars[i]   = theta_bar;
            h_V_bars[i]       = V_bar;
        }

        // 4) Copy to Device
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

        V_bars.resize(h_V_bars.size());
        V_bars.view().copy_from(h_V_bars.data());
    }

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(stencils.size());  // Each quad has 1 energy
        info.gradient_count(stencils.size() * StencilSize);  // Each quad has 4 vertices

        if(info.gradient_only())
            return;

        info.hessian_count(stencils.size() * HalfHessianSize);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace DSB = sym::discrete_shell_bending;

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

                       Float E = DSB::E(x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       energies(I) = E * V_bar * dt * dt;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace DSB = sym::discrete_shell_bending;

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

                       DSB::dEdx(G12, x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       G12 *= Vdt2;
                       DoubletVectorAssembler DVA{G3s};
                       DVA.segment<StencilSize>(I * StencilSize).write(stencil, G12);

                       if(gradient_only)
                           return;

                       DSB::ddEddx(H12x12, x0, x1, x2, x3, L0, h_bar, theta_bar, kappa);
                       H12x12 *= Vdt2;
                       make_spd(H12x12);

                       TripletMatrixAssembler TMA{H3x3s};
                       TMA.half_block<StencilSize>(I * HalfHessianSize).write(stencil, H12x12);
                   });
    }
};


REGISTER_SIM_SYSTEM(DiscreteShellBending);
}  // namespace uipc::backend::cuda
