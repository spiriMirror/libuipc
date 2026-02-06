#include <inter_primitive_effect_system/inter_primitive_constitution.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_stitch_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
class SoftVertexStitch : public InterPrimitiveConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 22;
    static constexpr SizeT StencilSize     = 2;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using InterPrimitiveConstitution::InterPrimitiveConstitution;

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    void do_build(BuildInfo& info) override {}

    muda::DeviceBuffer<Vector2i> topos;
    muda::DeviceBuffer<Float>    kappas;
    muda::DeviceBuffer<Float>    rest_lengths;

    vector<Vector2i> h_topos;
    vector<Float>    h_kappas;
    vector<Float>    h_rest_lengths;

    muda::CBufferView<Float>              energies;
    muda::CDoubletVectorView<Float, 3>    gradients;
    muda::CTripletMatrixView<Float, 3, 3> hessians;

    void do_init(FilteredInfo& info) override
    {
        list<Vector2i> topo_buffer;
        list<Float>    kappa_buffer;
        list<Float>    rest_length_buffer;

        auto geo_slots    = world().scene().geometries();
        using ForEachInfo = InterPrimitiveConstitutionManager::ForEachInfo;
        info.for_each(
            geo_slots,
            [&](const ForEachInfo& I, geometry::Geometry& geo)
            {
                auto topo = geo.instances().find<Vector2i>(builtin::topo);
                UIPC_ASSERT(topo, "SoftVertexStitch requires attribute `topo` on instances()");
                auto geo_ids = geo.meta().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "SoftVertexStitch requires attribute `geo_ids` on meta()");
                auto kappa = geo.instances().find<Float>("kappa");
                UIPC_ASSERT(kappa, "SoftVertexStitch requires attribute `kappa` on instances()");
                auto rest_length = geo.instances().find<Float>("rest_length");
                UIPC_ASSERT(rest_length, "SoftVertexStitch requires attribute `rest_length` on instances()");
                Vector2i ids = geo_ids->view()[0];

                auto l_slot = info.geo_slot(ids[0]);
                auto r_slot = info.geo_slot(ids[1]);

                auto l_geo = l_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(l_geo,
                            "SoftVertexStitch requires simplicial complex geometry, but yours {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                auto r_geo = r_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(r_geo,
                            "SoftVertexStitch requires simplicial complex geometry, but yours {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());

                auto l_offset = l_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(l_offset,
                            "SoftVertexStitch requires attribute `global_vertex_offset` on meta() of geometry {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                IndexT l_offset_v = l_offset->view()[0];
                auto r_offset = r_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(r_offset,
                            "SoftVertexStitch requires attribute `global_vertex_offset` on meta() of geometry {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());
                IndexT r_offset_v = r_offset->view()[0];

                auto topo_view = topo->view();
                for(auto& v : topo_view)
                {
                    topo_buffer.push_back(Vector2i{v[0] + l_offset_v, v[1] + r_offset_v});
                }

                auto kappa_view = kappa->view();
                for(auto kappa : kappa_view)
                {
                    kappa_buffer.push_back(kappa);
                }

                auto rest_length_view = rest_length->view();
                for(auto rl : rest_length_view)
                {
                    UIPC_ASSERT(rl >= 0, "rest_length must be non-negative");
                    rest_length_buffer.push_back(rl);
                }
            });

        h_topos.resize(topo_buffer.size());
        std::ranges::move(topo_buffer, h_topos.begin());
        topos.copy_from(h_topos);

        h_kappas.resize(kappa_buffer.size());
        std::ranges::move(kappa_buffer, h_kappas.begin());
        kappas.copy_from(h_kappas);

        h_rest_lengths.resize(rest_length_buffer.size());
        std::ranges::move(rest_length_buffer, h_rest_lengths.begin());
        rest_lengths.copy_from(h_rest_lengths);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(topos.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;

        energies = info.energies();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(topos.size(),
                   [topos        = topos.cviewer().name("topos"),
                    xs           = info.positions().cviewer().name("xs"),
                    kappas       = kappas.cviewer().name("kappas"),
                    rest_lengths = rest_lengths.cviewer().name("rest_lengths"),
                    Es           = info.energies().viewer().name("Es"),
                    dt           = info.dt()] __device__(int I)
                   {
                       const Vector2i& PP   = topos(I);
                       Float           Kt2  = kappas(I) * dt * dt;
                       Float           dist = (xs(PP[0]) - xs(PP[1])).norm();
                       Float           diff = dist - rest_lengths(I);
                       Es(I)                = 0.5 * Kt2 * diff * diff;
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(StencilSize * topos.size());

        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * topos.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace SVS = sym::soft_vertex_stitch;

        gradients = info.gradients();
        hessians  = info.hessians();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(topos.size(),
                   [topos         = topos.cviewer().name("topos"),
                    xs            = info.positions().cviewer().name("xs"),
                    kappas        = kappas.cviewer().name("kappas"),
                    rest_lengths  = rest_lengths.cviewer().name("rest_lengths"),
                    G3s           = info.gradients().viewer().name("Gs"),
                    H3x3s         = info.hessians().viewer().name("H3x3s"),
                    dt            = info.dt(),
                    gradient_only = info.gradient_only()] __device__(int I)
                   {
                       Vector6         X;
                       const Vector2i& PP = topos(I);
                       for(int i = 0; i < 2; ++i)
                           X.segment<3>(3 * i) = xs(PP[i]);

                       Float   L0  = rest_lengths(I);
                       Float   Kt2 = kappas(I) * dt * dt;
                       Vector6 G;
                       SVS::dEdX(G, Kt2, X, L0);
                       DoubletVectorAssembler VA{G3s};
                       VA.segment<StencilSize>(I * StencilSize).write(PP, G);

                       if(gradient_only)
                           return;

                       Matrix6x6 H;
                       SVS::ddEddX(H, Kt2, X, L0);
                       make_spd(H);
                       TripletMatrixAssembler MA{H3x3s};
                       MA.half_block<StencilSize>(I * HalfHessianSize).write(PP, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftVertexStitch);
}  // namespace uipc::backend::cuda
