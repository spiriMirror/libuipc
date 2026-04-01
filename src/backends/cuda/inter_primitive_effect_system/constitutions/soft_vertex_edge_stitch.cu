#include <inter_primitive_effect_system/inter_primitive_constitution.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_edge_stitch_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/inverse.h>

namespace uipc::backend::cuda
{
class SoftVertexEdgeStitch : public InterPrimitiveConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 29;
    static constexpr SizeT StencilSize     = 3;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using InterPrimitiveConstitution::InterPrimitiveConstitution;

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    void do_build(BuildInfo& info) override {}

    muda::DeviceBuffer<Vector3i>  topos;
    muda::DeviceBuffer<Float>     mus;
    muda::DeviceBuffer<Float>     lambdas;
    muda::DeviceBuffer<Matrix2x2> inv_Bs;
    muda::DeviceBuffer<Float>     rest_areas;
    muda::DeviceBuffer<Float>     thicknesses;

    vector<Vector3i>  h_topos;
    vector<Float>     h_mus;
    vector<Float>     h_lambdas;
    vector<Matrix2x2> h_inv_Bs;
    vector<Float>     h_rest_areas;
    vector<Float>     h_thicknesses;

    muda::CBufferView<Float>              energies;
    muda::CDoubletVectorView<Float, 3>    gradients;
    muda::CTripletMatrixView<Float, 3, 3> hessians;

    void do_init(FilteredInfo& info) override
    {
        list<Vector3i>  topo_buffer;
        list<Float>     mu_buffer;
        list<Float>     lambda_buffer;
        list<Matrix2x2> inv_B_buffer;
        list<Float>     rest_area_buffer;
        list<Float>     thickness_buffer;

        auto geo_slots    = world().scene().geometries();
        using ForEachInfo = InterPrimitiveConstitutionManager::ForEachInfo;
        info.for_each(
            geo_slots,
            [&](const ForEachInfo& I, geometry::Geometry& geo)
            {
                auto topo = geo.instances().find<Vector3i>(builtin::topo);
                UIPC_ASSERT(topo,
                            "SoftVertexEdgeStitch requires attribute `topo` on instances()");
                auto geo_ids = geo.meta().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids,
                            "SoftVertexEdgeStitch requires attribute `geo_ids` on meta()");
                auto mu_slot = geo.instances().find<Float>("mu");
                UIPC_ASSERT(mu_slot,
                            "SoftVertexEdgeStitch requires attribute `mu` on instances()");
                auto lambda_slot = geo.instances().find<Float>("lambda");
                UIPC_ASSERT(lambda_slot,
                            "SoftVertexEdgeStitch requires attribute `lambda` on instances()");
                auto thick_slot = geo.instances().find<Float>("thickness");
                UIPC_ASSERT(thick_slot,
                            "SoftVertexEdgeStitch requires attribute `thickness` on instances()");
                auto min_sep_slot = geo.instances().find<Float>("min_separate_distance");
                UIPC_ASSERT(min_sep_slot,
                            "SoftVertexEdgeStitch requires attribute `min_separate_distance` on instances()");

                Vector2i ids         = geo_ids->view()[0];
                auto     l_slot      = info.geo_slot(ids[0]);
                auto     r_slot      = info.geo_slot(ids[1]);
                auto     l_rest_slot = info.rest_geo_slot(ids[0]);
                auto     r_rest_slot = info.rest_geo_slot(ids[1]);
                UIPC_ASSERT(l_rest_slot,
                            "SoftVertexEdgeStitch requires rest geometry for slot id {}",
                            ids[0]);
                UIPC_ASSERT(r_rest_slot,
                            "SoftVertexEdgeStitch requires rest geometry for slot id {}",
                            ids[1]);

                auto l_geo = l_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(l_geo,
                            "SoftVertexEdgeStitch requires simplicial complex, but got {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                auto r_geo = r_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(r_geo,
                            "SoftVertexEdgeStitch requires simplicial complex, but got {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());
                auto l_rest_geo =
                    l_rest_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(l_rest_geo,
                            "SoftVertexEdgeStitch requires rest simplicial complex for id {}",
                            ids[0]);
                auto r_rest_geo =
                    r_rest_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(r_rest_geo,
                            "SoftVertexEdgeStitch requires rest simplicial complex for id {}",
                            ids[1]);

                auto l_offset =
                    l_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(l_offset,
                            "SoftVertexEdgeStitch requires `global_vertex_offset` on meta() of geometry {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                IndexT l_offset_v = l_offset->view()[0];
                auto   r_offset =
                    r_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(r_offset,
                            "SoftVertexEdgeStitch requires `global_vertex_offset` on meta() of geometry {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());
                IndexT r_offset_v = r_offset->view()[0];

                auto aim0_pos     = l_geo->positions().view();
                auto aim1_pos     = r_geo->positions().view();

                Transform l_transform(l_geo->transforms().view()[0]);
                Transform r_transform(r_geo->transforms().view()[0]);

                auto topo_view    = topo->view();
                auto mu_view      = mu_slot->view();
                auto lambda_view  = lambda_slot->view();
                auto thick_view   = thick_slot->view();
                auto min_sep_view = min_sep_slot->view();
                SizeT n           = topo_view.size();

                for(SizeT i = 0; i < n; ++i)
                {
                    const Vector3i& t = topo_view[i];
                    IndexT  v_id = t(0), e0 = t(1), e1 = t(2);
                    Vector3 x0 = l_transform * aim0_pos[v_id];
                    Vector3 x1 = r_transform * aim1_pos[e0];
                    Vector3 x2 = r_transform * aim1_pos[e1];

                    constexpr Float geo_degeneracy_tol = 1e-12;

                    Float   d    = min_sep_view[i];
                    Vector3 edge = x2 - x1;
                    Float   edge_len = edge.norm();
                    UIPC_ASSERT(edge_len >= geo_degeneracy_tol,
                                "SoftVertexEdgeStitch: edge ({},{}) is degenerate",
                                e0, e1);
                    Vector3 edge_dir = edge / edge_len;

                    // project vertex onto edge line to get closest point
                    Float   proj = edge_dir.dot(x0 - x1);
                    proj         = std::clamp(proj, Float(0), edge_len);
                    Vector3 closest = x1 + proj * edge_dir;
                    Vector3 ve_dir  = x0 - closest;
                    Float   ve_dist = ve_dir.norm();

                    if(ve_dist < d)
                    {
                        if(ve_dist < geo_degeneracy_tol)
                        {
                            // collinear: pick arbitrary perpendicular
                            Vector3 arbitrary =
                                std::abs(edge_dir(0)) < 0.9
                                    ? Vector3{1, 0, 0}
                                    : Vector3{0, 1, 0};
                            ve_dir = edge_dir.cross(arbitrary);
                            ve_dir.normalize();
                        }
                        else
                        {
                            ve_dir /= ve_dist;
                        }
                        x0 = closest + d * ve_dir;
                    }

                    // compute rest metric B and its inverse
                    Vector3   e01 = x1 - x0;
                    Vector3   e02 = x2 - x0;
                    Matrix2x2 B;
                    B(0, 0) = e01.dot(e01);
                    B(0, 1) = e01.dot(e02);
                    B(1, 0) = B(0, 1);
                    B(1, 1) = e02.dot(e02);
                    Matrix2x2 IB = B.inverse();

                    // rest area = 0.5 * |e01 x e02|
                    Float rest_area =
                        0.5 * e01.cross(e02).norm();

                    topo_buffer.push_back(Vector3i{t(0) + l_offset_v,
                                                   t(1) + r_offset_v,
                                                   t(2) + r_offset_v});
                    mu_buffer.push_back(mu_view[i]);
                    lambda_buffer.push_back(lambda_view[i]);
                    inv_B_buffer.push_back(IB);
                    rest_area_buffer.push_back(rest_area);
                    thickness_buffer.push_back(thick_view[i]);
                }
            });

        h_topos.assign(topo_buffer.begin(), topo_buffer.end());
        h_mus.assign(mu_buffer.begin(), mu_buffer.end());
        h_lambdas.assign(lambda_buffer.begin(), lambda_buffer.end());
        h_inv_Bs.assign(inv_B_buffer.begin(), inv_B_buffer.end());
        h_rest_areas.assign(rest_area_buffer.begin(), rest_area_buffer.end());
        h_thicknesses.assign(thickness_buffer.begin(), thickness_buffer.end());

        topos.copy_from(h_topos);
        mus.copy_from(h_mus);
        lambdas.copy_from(h_lambdas);
        inv_Bs.copy_from(h_inv_Bs);
        rest_areas.copy_from(h_rest_areas);
        thicknesses.copy_from(h_thicknesses);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(topos.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace NH = sym::soft_vertex_edge_stitch;

        energies = info.energies();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(topos.size(),
                   [topos       = topos.cviewer().name("topos"),
                    xs          = info.positions().cviewer().name("xs"),
                    mus         = mus.cviewer().name("mus"),
                    lambdas     = lambdas.cviewer().name("lambdas"),
                    IBs         = inv_Bs.cviewer().name("IBs"),
                    rest_areas  = rest_areas.cviewer().name("rest_areas"),
                    thick       = thicknesses.cviewer().name("thicknesses"),
                    Es          = info.energies().viewer().name("Es"),
                    dt          = info.dt()] __device__(int I)
                   {
                       const Vector3i& tri = topos(I);
                       Vector9         X;
                       for(int k = 0; k < 3; ++k)
                           X.segment<3>(3 * k) = xs(tri(k));

                       Float E_val;
                       NH::E(E_val, lambdas(I), mus(I), X, IBs(I));

                       Float Vdt2 = rest_areas(I) * 2 * thick(I) * dt * dt;
                       Es(I)      = E_val * Vdt2;
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
        namespace NH = sym::soft_vertex_edge_stitch;

        gradients = info.gradients();
        hessians  = info.hessians();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(topos.size(),
                   [topos         = topos.cviewer().name("topos"),
                    xs            = info.positions().cviewer().name("xs"),
                    mus           = mus.cviewer().name("mus"),
                    lambdas       = lambdas.cviewer().name("lambdas"),
                    IBs           = inv_Bs.cviewer().name("IBs"),
                    rest_areas    = rest_areas.cviewer().name("rest_areas"),
                    thick         = thicknesses.cviewer().name("thicknesses"),
                    G3s           = info.gradients().viewer().name("Gs"),
                    H3x3s        = info.hessians().viewer().name("H3x3s"),
                    dt            = info.dt(),
                    gradient_only = info.gradient_only()] __device__(int I) mutable
                   {
                       const Vector3i& tri = topos(I);
                       Vector9         X;
                       for(int k = 0; k < 3; ++k)
                           X.segment<3>(3 * k) = xs(tri(k));

                       Float Vdt2 = rest_areas(I) * 2 * thick(I) * dt * dt;

                       Vector9 G;
                       NH::dEdX(G, lambdas(I), mus(I), X, IBs(I));
                       G *= Vdt2;

                       DoubletVectorAssembler VA{G3s};
                       VA.segment<StencilSize>(I * StencilSize).write(tri, G);

                       if(gradient_only)
                           return;

                       Matrix9x9 H;
                       NH::ddEddX(H, lambdas(I), mus(I), X, IBs(I));
                       make_spd(H);
                       H *= Vdt2;

                       TripletMatrixAssembler MA{H3x3s};
                       MA.half_block<StencilSize>(I * HalfHessianSize)
                           .write(tri, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftVertexEdgeStitch);
}  // namespace uipc::backend::cuda
