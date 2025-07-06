#include <contact_system/inter_primitive_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
class SoftVertexStitch : public InterPrimitiveConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 22;

    using InterPrimitiveConstitution::InterPrimitiveConstitution;

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    void do_build(BuildInfo& info) override {}

    muda::DeviceBuffer<Vector2i> topos;
    muda::DeviceBuffer<Float>    kappas;

    vector<Vector2i> h_topos;
    vector<Float>    h_kappas;

    muda::CBufferView<Float>              energies;
    muda::CDoubletVectorView<Float, 3>    gradients;
    muda::CTripletMatrixView<Float, 3, 3> hessians;

    void do_init(FilteredInfo& info) override
    {
        list<Vector2i> topo_buffer;
        list<Float>    kappa_buffer;

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
            });

        h_topos.resize(topo_buffer.size());
        std::ranges::move(topo_buffer, h_topos.begin());
        topos.copy_from(h_topos);

        h_kappas.resize(kappa_buffer.size());
        std::ranges::move(kappa_buffer, h_kappas.begin());
        kappas.copy_from(h_kappas);
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
                   [topos  = topos.cviewer().name("topos"),
                    xs     = info.positions().cviewer().name("xs"),
                    kappas = kappas.cviewer().name("kappas"),
                    Es     = info.energies().viewer().name("Es"),
                    dt     = info.dt()] __device__(int I)
                   {
                       const Vector2i& PP  = topos(I);
                       Float           Kt2 = kappas(I) * dt * dt;
                       Es(I) = 0.5 * Kt2 * (xs(PP[0]) - xs(PP[1])).squaredNorm();
                   });
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_segment_count(2 * topos.size());
        info.hessian_block_count(4 * topos.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;

        gradients = info.gradients();
        hessians  = info.hessians();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(topos.size(),
                   [topos  = topos.cviewer().name("topos"),
                    xs     = info.positions().cviewer().name("xs"),
                    kappas = kappas.cviewer().name("kappas"),
                    G3s    = info.gradients().viewer().name("Gs"),
                    H3x3s  = info.hessians().viewer().name("H3x3s"),
                    dt     = info.dt()] __device__(int I)
                   {
                       const Vector2i& PP  = topos(I);
                       Float           Kt2 = kappas(I) * dt * dt;
                       Vector3         dX  = xs(PP[0]) - xs(PP[1]);

                       Vector3   G = Kt2 * dX;
                       Matrix3x3 H = Kt2 * Matrix3x3::Identity();
                       // gradient
                       G3s(2 * I + 0).write(PP[0], G);
                       G3s(2 * I + 1).write(PP[1], -G);

                       // hessian
                       H3x3s(4 * I + 0).write(PP[0], PP[0], H);
                       H3x3s(4 * I + 1).write(PP[0], PP[1], -H);
                       H3x3s(4 * I + 2).write(PP[1], PP[0], -H);
                       H3x3s(4 * I + 3).write(PP[1], PP[1], H);
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftVertexStitch);

class SoftVertexStitchContactExporter : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SoftVertexStitch> soft_vertex_stitch;

    std::string prim_type = fmt::format("#{}", SoftVertexStitch::ConstitutionUID);

    void do_build(BuildInfo& info) override
    {
        soft_vertex_stitch = require<SoftVertexStitch>();
    }

    std::string_view get_prim_type() const noexcept override
    {
        return prim_type;
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& energy_geo) override
    {
        auto& topos    = soft_vertex_stitch->topos;
        auto& energies = soft_vertex_stitch->energies;

        energy_geo.instances().resize(topos.size());

        UIPC_ASSERT(topos.size() == energies.size(),
                    "topos size ({}) and energies size ({}) must match.",
                    topos.size(),
                    energies.size());

        auto topo = energy_geo.instances().find<Vector2i>("topo");
        if(!topo)
        {
            topo = energy_geo.instances().create<Vector2i>("topo");
        }
        auto topo_view = view(*topo);
        topos.view().copy_to(topo_view.data());

        auto energy = energy_geo.instances().find<Float>("energy");
        if(!energy)
        {
            energy = energy_geo.instances().create<Float>("energy");
        }
        auto energy_view = view(*energy);
        energies.copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto& grads = soft_vertex_stitch->gradients;
        vert_grad.instances().resize(grads.doublet_count());

        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i");
        }
        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad");
        }
        auto i_view    = view(*i);
        auto grad_view = view(*grad);

        grads.indices().copy_to(i_view.data());
        grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto& hess = soft_vertex_stitch->hessians;
        vert_hess.instances().resize(hess.triplet_count());

        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i");
        }
        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j");
        }
        auto hess_view = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess_view)
        {
            hess_view = vert_hess.instances().create<Matrix3x3>("hess");
        }
        auto i_view    = view(*i);
        auto j_view    = view(*j);
        auto hess_data = view(*hess_view);

        hess.row_indices().copy_to(i_view.data());
        hess.col_indices().copy_to(j_view.data());
        hess.values().copy_to(hess_data.data());
    }
};

REGISTER_SIM_SYSTEM(SoftVertexStitchContactExporter);
}  // namespace uipc::backend::cuda
