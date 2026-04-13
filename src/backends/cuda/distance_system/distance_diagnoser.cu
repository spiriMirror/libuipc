#include <distance_system/distance_diagnoser.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance/edge_edge_mollifier.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>
#include <muda/buffer/device_buffer.h>
#include <muda/launch/parallel_for.h>
#include <uipc/common/enumerate.h>
#include <string_view>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DistanceDiagnoser);

// =====================================================================
// Overrider
// =====================================================================
DistanceDiagnoserFeatureOverrider::DistanceDiagnoserFeatureOverrider(DistanceDiagnoser* diagnoser)
{
    UIPC_ASSERT(diagnoser, "DistanceDiagnoser must not be null");
    m_diagnoser = *diagnoser;
}

void DistanceDiagnoserFeatureOverrider::do_compute_point_triangle_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& triangles)
{
    m_diagnoser->compute_point_triangle_distance(R, points, triangles);
}

void DistanceDiagnoserFeatureOverrider::do_compute_edge_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& edges_a,
    const geometry::SimplicialComplex& edges_b,
    const geometry::SimplicialComplex& rest_edges_a,
    const geometry::SimplicialComplex& rest_edges_b)
{
    m_diagnoser->compute_edge_edge_distance(R, edges_a, edges_b, rest_edges_a, rest_edges_b);
}

void DistanceDiagnoserFeatureOverrider::do_compute_point_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& edges)
{
    m_diagnoser->compute_point_edge_distance(R, points, edges);
}

void DistanceDiagnoserFeatureOverrider::do_compute_point_point_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points_a,
    const geometry::SimplicialComplex& points_b)
{
    m_diagnoser->compute_point_point_distance(R, points_a, points_b);
}

// =====================================================================
// SimSystem
// =====================================================================
void DistanceDiagnoser::do_build()
{
    auto overrider = std::make_shared<DistanceDiagnoserFeatureOverrider>(this);
    auto feature   = std::make_shared<core::DistanceDiagnoserFeature>(overrider);
    features().insert(feature);
}

namespace detail
{
    template <typename T>
    MUDA_GENERIC void point_triangle_closest_point_coord(
        const Vector4i&            flag,
        const Eigen::Vector<T, 3>& p,
        const Eigen::Vector<T, 3>& t0,
        const Eigen::Vector<T, 3>& t1,
        const Eigen::Vector<T, 3>& t2,
        Eigen::Vector<T, 4>&       coord)
    {
        coord.setZero();
        IndexT dim = distance::detail::active_count(flag);

        if(dim == 2)
        {
            if(flag[1]) coord[1] = T(1);
            else if(flag[2]) coord[2] = T(1);
            else if(flag[3]) coord[3] = T(1);
        }
        else if(dim == 3)
        {
            Eigen::Vector<T, 3> e0, e1;
            IndexT idx0 = -1;
            IndexT idx1 = -1;
            if(flag[1] && flag[2])      { e0 = t0; e1 = t1; idx0 = 1; idx1 = 2; }
            else if(flag[2] && flag[3]) { e0 = t1; e1 = t2; idx0 = 2; idx1 = 3; }
            else if(flag[3] && flag[1]) { e0 = t2; e1 = t0; idx0 = 3; idx1 = 1; }

            Eigen::Vector<T, 3> edge = e1 - e0;
            T len2 = edge.squaredNorm();
            T t_param = T(0);
            if(len2 > T(0))
                t_param = (p - e0).dot(edge) / len2;
            t_param = max(T(0), min(T(1), t_param));

            coord[idx0] = T(1) - t_param;
            coord[idx1] = t_param;
        }
        else if(dim == 4)
        {
            Eigen::Vector<T, 3> v0 = t1 - t0;
            Eigen::Vector<T, 3> v1 = t2 - t0;
            Eigen::Vector<T, 3> v2 = p - t0;

            T d00 = v0.dot(v0);
            T d01 = v0.dot(v1);
            T d11 = v1.dot(v1);
            T d20 = v2.dot(v0);
            T d21 = v2.dot(v1);

            T denom = d00 * d11 - d01 * d01;
            T v = (d11 * d20 - d01 * d21) / denom;
            T w = (d00 * d21 - d01 * d20) / denom;
            T u = T(1) - v - w;

            coord[1] = u;
            coord[2] = v;
            coord[3] = w;
        }
    }

    template <typename T>
    MUDA_GENERIC void edge_edge_closest_point_coord(
        const Vector4i&            flag,
        const Eigen::Vector<T, 3>& ea0,
        const Eigen::Vector<T, 3>& ea1,
        const Eigen::Vector<T, 3>& eb0,
        const Eigen::Vector<T, 3>& eb1,
        Eigen::Vector<T, 4>&       coord)
    {
        coord.setZero();
        IndexT dim = distance::detail::active_count(flag);

        if(dim == 2)
        {
            if(flag[0]) coord[0] = T(1);
            else        coord[1] = T(1);

            if(flag[2]) coord[2] = T(1);
            else        coord[3] = T(1);
        }
        else if(dim == 3)
        {
            Eigen::Vector<T, 3> point, e0, e1;
            IndexT pt_idx = -1;
            IndexT e0_idx = -1;
            IndexT e1_idx = -1;

            if(!flag[0])
            {
                point = ea1; pt_idx = 1;
                e0 = eb0; e0_idx = 2;
                e1 = eb1; e1_idx = 3;
            }
            else if(!flag[1])
            {
                point = ea0; pt_idx = 0;
                e0 = eb0; e0_idx = 2;
                e1 = eb1; e1_idx = 3;
            }
            else if(!flag[2])
            {
                point = eb1; pt_idx = 3;
                e0 = ea0; e0_idx = 0;
                e1 = ea1; e1_idx = 1;
            }
            else
            {
                point = eb0; pt_idx = 2;
                e0 = ea0; e0_idx = 0;
                e1 = ea1; e1_idx = 1;
            }

            coord[pt_idx] = T(1);

            Eigen::Vector<T, 3> edge = e1 - e0;
            T len2 = edge.squaredNorm();
            T t_param = T(0);
            if(len2 > T(0))
                t_param = (point - e0).dot(edge) / len2;
            t_param = max(T(0), min(T(1), t_param));

            coord[e0_idx] = T(1) - t_param;
            coord[e1_idx] = t_param;
        }
        else if(dim == 4)
        {
            Eigen::Vector<T, 3> u = ea1 - ea0;
            Eigen::Vector<T, 3> v = eb1 - eb0;
            Eigen::Vector<T, 3> w = ea0 - eb0;

            T a = u.dot(u);
            T b = u.dot(v);
            T c = v.dot(v);
            T d = u.dot(w);
            T e = v.dot(w);
            T D = a * c - b * b;

            T s = T(0), t = T(0);
            if(D > T(1e-30))
            {
                s = (b * e - c * d) / D;
                t = (a * e - b * d) / D;
            }
            s = max(T(0), min(T(1), s));
            t = max(T(0), min(T(1), t));

            coord[0] = T(1) - s;
            coord[1] = s;
            coord[2] = T(1) - t;
            coord[3] = t;
        }
    }

    template <typename T>
    MUDA_GENERIC void point_edge_closest_point_coord(
        const Vector3i&            flag,
        const Eigen::Vector<T, 3>& p,
        const Eigen::Vector<T, 3>& e0,
        const Eigen::Vector<T, 3>& e1,
        Eigen::Vector<T, 3>&       coord)
    {
        coord.setZero();
        IndexT dim = distance::detail::active_count(flag);

        if(dim == 2)
        {
            if(flag[1]) coord[1] = T(1);
            else        coord[2] = T(1);
        }
        else if(dim == 3)
        {
            Eigen::Vector<T, 3> edge = e1 - e0;
            T len2 = edge.squaredNorm();
            T t_param = T(0);
            if(len2 > T(0))
                t_param = (p - e0).dot(edge) / len2;
            t_param = max(T(0), min(T(1), t_param));

            coord[1] = T(1) - t_param;
            coord[2] = t_param;
        }
    }

    template <typename T>
    void upload_vertex_attribute(muda::DeviceBuffer<T>&             dst,
                                 const geometry::SimplicialComplex& sc,
                                 std::string_view                   name)
    {
        auto attr = sc.vertices().find<T>(std::string{name});
        UIPC_ASSERT(attr,
                     "Vertex attribute '{}' not found on SimplicialComplex",
                     name);
        auto v = attr->view();
        dst.resize(v.size());
        dst.view().copy_from(v.data());
    }

    template <typename T, typename D>
    void copy_back(geometry::Geometry&    R,
                   std::string_view       name,
                   const D&               default_val,
                   muda::BufferView<T>    buf)
    {
        std::string n{name};
        auto attr = R.instances().find<T>(n);
        if(!attr)
            attr = R.instances().create<T>(n, T(default_val));
        buf.copy_to(view(*attr).data());
    }
}  // namespace detail

// =====================================================================
// Point-Triangle Distance (+ Barrier)
// =====================================================================
void DistanceDiagnoser::compute_point_triangle_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& triangles)
{
    using namespace muda;

    auto h_point_pos = points.positions().view();
    auto h_tri_pos   = triangles.positions().view();
    auto h_tri_topo  = triangles.triangles().topo().view();

    SizeT num_points    = h_point_pos.size();
    SizeT num_triangles = h_tri_topo.size();
    SizeT num_pairs     = num_points * num_triangles;

    if(num_pairs == 0)
    {
        R.instances().resize(0);
        return;
    }

    DeviceBuffer<Vector3>  point_pos(num_points);
    DeviceBuffer<Vector3>  tri_pos(h_tri_pos.size());
    DeviceBuffer<Vector3i> tri_topo(num_triangles);
    DeviceBuffer<Float>    dhat_p;
    DeviceBuffer<Float>    dhat_t;
    DeviceBuffer<Float>    thickness_p;
    DeviceBuffer<Float>    thickness_t;

    point_pos.view().copy_from(h_point_pos.data());
    tri_pos.view().copy_from(h_tri_pos.data());
    tri_topo.view().copy_from(h_tri_topo.data());
    detail::upload_vertex_attribute(dhat_p, points,    "d_hat");
    detail::upload_vertex_attribute(dhat_t, triangles, "d_hat");
    detail::upload_vertex_attribute(thickness_p,   points,    "thickness");
    detail::upload_vertex_attribute(thickness_t,   triangles, "thickness");

    DeviceBuffer<Float>        dist2(num_pairs);
    DeviceBuffer<Vector12>     dist2_grad(num_pairs);
    DeviceBuffer<Matrix12x12>  dist2_hess(num_pairs);
    DeviceBuffer<Vector4i>     flags(num_pairs);
    DeviceBuffer<Vector4>      coords(num_pairs);
    DeviceBuffer<Float>        barrier(num_pairs);
    DeviceBuffer<Vector12>     barrier_grad(num_pairs);
    DeviceBuffer<Matrix12x12>  barrier_hess(num_pairs);

    SizeT M = num_triangles;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [point_pos    = point_pos.viewer().name("point_pos"),
                tri_pos      = tri_pos.viewer().name("tri_pos"),
                tri_topo     = tri_topo.viewer().name("tri_topo"),
                dhat_p       = dhat_p.cviewer().name("dhat_p"),
                dhat_t       = dhat_t.cviewer().name("dhat_t"),
                thickness_p         = thickness_p.cviewer().name("thickness_p"),
                thickness_t         = thickness_t.cviewer().name("thickness_t"),
                dist2        = dist2.viewer().name("dist2"),
                dist2_grad   = dist2_grad.viewer().name("dist2_grad"),
                dist2_hess   = dist2_hess.viewer().name("dist2_hess"),
                flags        = flags.viewer().name("flags"),
                coords       = coords.viewer().name("coords"),
                barrier      = barrier.viewer().name("barrier"),
                barrier_grad = barrier_grad.viewer().name("barrier_grad"),
                barrier_hess = barrier_hess.viewer().name("barrier_hess"),
                M] __device__(int i) mutable
               {
                   IndexT pi = i / M;
                   IndexT ti = i % M;

                   const Vector3& P  = point_pos(pi);
                   Vector3i       TI = tri_topo(ti);
                   const Vector3& T0 = tri_pos(TI[0]);
                   const Vector3& T1 = tri_pos(TI[1]);
                   const Vector3& T2 = tri_pos(TI[2]);

                   Vector4i flag = distance::point_triangle_distance_flag(P, T0, T1, T2);

                   Float D;
                   distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

                   Vector12 GradD;
                   distance::point_triangle_distance2_gradient(flag, P, T0, T1, T2, GradD);

                   Matrix12x12 HessD;
                   distance::point_triangle_distance2_hessian(flag, P, T0, T1, T2, HessD);

                   Vector4 coord;
                   detail::point_triangle_closest_point_coord(flag, P, T0, T1, T2, coord);

                   Float d_hat_pair = PT_d_hat(dhat_p(pi), dhat_t(TI[0]), dhat_t(TI[1]), dhat_t(TI[2]));
                   Float xi         = PT_thickness(thickness_p(pi), thickness_t(TI[0]), thickness_t(TI[1]), thickness_t(TI[2]));
                   Vector2 dr = D_range(xi, d_hat_pair);

                   Float B = Float(0);
                   Vector12 GradB = Vector12::Zero();
                   Matrix12x12 HessB = Matrix12x12::Zero();

                   if(is_active_D(dr, D))
                   {
                       Float kappa = Float(1);
                       sym::codim_ipc_contact::KappaBarrier(B, kappa, D, d_hat_pair, xi);
                       Float dBdD;
                       sym::codim_ipc_contact::dKappaBarrierdD(dBdD, kappa, D, d_hat_pair, xi);
                       Float ddBddD;
                       sym::codim_ipc_contact::ddKappaBarrierddD(ddBddD, kappa, D, d_hat_pair, xi);
                       GradB = dBdD * GradD;
                       HessB = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
                   }

                   dist2(i)        = D;
                   dist2_grad(i)   = GradD;
                   dist2_hess(i)   = HessD;
                   flags(i)        = flag;
                   coords(i)       = coord;
                   barrier(i)      = B;
                   barrier_grad(i) = GradB;
                   barrier_hess(i) = HessB;
               });

    R.instances().resize(num_pairs);

    detail::copy_back(R, "dist2",        Float(0),            dist2.view());
    detail::copy_back(R, "dist2/grad",   Vector12::Zero(),    dist2_grad.view());
    detail::copy_back(R, "dist2/hess",   Matrix12x12::Zero(), dist2_hess.view());
    detail::copy_back(R, "flag",         Vector4i::Zero(),    flags.view());
    detail::copy_back(R, "coord",        Vector4::Zero(),     coords.view());
    detail::copy_back(R, "barrier",      Float(0),            barrier.view());
    detail::copy_back(R, "barrier/grad", Vector12::Zero(),    barrier_grad.view());
    detail::copy_back(R, "barrier/hess", Matrix12x12::Zero(), barrier_hess.view());
}

// =====================================================================
// Edge-Edge Distance (with Mollifier + Barrier)
// =====================================================================
void DistanceDiagnoser::compute_edge_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& edges_a,
    const geometry::SimplicialComplex& edges_b,
    const geometry::SimplicialComplex& rest_edges_a,
    const geometry::SimplicialComplex& rest_edges_b)
{
    using namespace muda;

    auto h_pos_a      = edges_a.positions().view();
    auto h_pos_b      = edges_b.positions().view();
    auto h_topo_a     = edges_a.edges().topo().view();
    auto h_topo_b     = edges_b.edges().topo().view();
    auto h_rest_pos_a = rest_edges_a.positions().view();
    auto h_rest_pos_b = rest_edges_b.positions().view();

    SizeT num_ea = h_topo_a.size();
    SizeT num_eb = h_topo_b.size();
    SizeT num_pairs = num_ea * num_eb;

    if(num_pairs == 0)
    {
        R.instances().resize(0);
        return;
    }

    auto mc_attr = R.instances().find<Float>("mollifier_coeff");
    Float mollifier_coeff_val = Float(1e-3);
    if(mc_attr)
    {
        auto mc_view = mc_attr->view();
        if(mc_view.size() > 0)
            mollifier_coeff_val = mc_view[0];
    }

    DeviceBuffer<Vector3>  pos_a(h_pos_a.size());
    DeviceBuffer<Vector3>  pos_b(h_pos_b.size());
    DeviceBuffer<Vector2i> topo_a(num_ea);
    DeviceBuffer<Vector2i> topo_b(num_eb);
    DeviceBuffer<Vector3>  rest_pos_a(h_rest_pos_a.size());
    DeviceBuffer<Vector3>  rest_pos_b(h_rest_pos_b.size());
    DeviceBuffer<Float>    dhat_a;
    DeviceBuffer<Float>    dhat_b;
    DeviceBuffer<Float>    thickness_a;
    DeviceBuffer<Float>    thickness_b;

    pos_a.view().copy_from(h_pos_a.data());
    pos_b.view().copy_from(h_pos_b.data());
    topo_a.view().copy_from(h_topo_a.data());
    topo_b.view().copy_from(h_topo_b.data());
    rest_pos_a.view().copy_from(h_rest_pos_a.data());
    rest_pos_b.view().copy_from(h_rest_pos_b.data());
    detail::upload_vertex_attribute(dhat_a, edges_a, "d_hat");
    detail::upload_vertex_attribute(dhat_b, edges_b, "d_hat");
    detail::upload_vertex_attribute(thickness_a,   edges_a, "thickness");
    detail::upload_vertex_attribute(thickness_b,   edges_b, "thickness");

    DeviceBuffer<Float>        dist2(num_pairs);
    DeviceBuffer<Vector12>     dist2_grad(num_pairs);
    DeviceBuffer<Matrix12x12>  dist2_hess(num_pairs);
    DeviceBuffer<Vector4i>     flags(num_pairs);
    DeviceBuffer<Vector4>      coords(num_pairs);
    DeviceBuffer<Float>        eps_xs(num_pairs);
    DeviceBuffer<Float>        e_ks(num_pairs);
    DeviceBuffer<Vector12>     ek_grad(num_pairs);
    DeviceBuffer<Matrix12x12>  ek_hess(num_pairs);
    DeviceBuffer<Float>        barrier(num_pairs);
    DeviceBuffer<Vector12>     barrier_grad(num_pairs);
    DeviceBuffer<Matrix12x12>  barrier_hess(num_pairs);

    SizeT M = num_eb;
    Float coeff = mollifier_coeff_val;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [pos_a        = pos_a.viewer().name("pos_a"),
                pos_b        = pos_b.viewer().name("pos_b"),
                topo_a       = topo_a.viewer().name("topo_a"),
                topo_b       = topo_b.viewer().name("topo_b"),
                rest_pos_a   = rest_pos_a.viewer().name("rest_pos_a"),
                rest_pos_b   = rest_pos_b.viewer().name("rest_pos_b"),
                dhat_a       = dhat_a.cviewer().name("dhat_a"),
                dhat_b       = dhat_b.cviewer().name("dhat_b"),
                thickness_a         = thickness_a.cviewer().name("thickness_a"),
                thickness_b         = thickness_b.cviewer().name("thickness_b"),
                dist2        = dist2.viewer().name("dist2"),
                dist2_grad   = dist2_grad.viewer().name("dist2_grad"),
                dist2_hess   = dist2_hess.viewer().name("dist2_hess"),
                flags        = flags.viewer().name("flags"),
                coords       = coords.viewer().name("coords"),
                eps_xs       = eps_xs.viewer().name("eps_xs"),
                e_ks         = e_ks.viewer().name("e_ks"),
                ek_grad      = ek_grad.viewer().name("ek_grad"),
                ek_hess      = ek_hess.viewer().name("ek_hess"),
                barrier      = barrier.viewer().name("barrier"),
                barrier_grad = barrier_grad.viewer().name("barrier_grad"),
                barrier_hess = barrier_hess.viewer().name("barrier_hess"),
                M, coeff] __device__(int i) mutable
               {
                   IndexT ai = i / M;
                   IndexT bi = i % M;

                   Vector2i EA = topo_a(ai);
                   Vector2i EB = topo_b(bi);

                   const Vector3& ea0 = pos_a(EA[0]);
                   const Vector3& ea1 = pos_a(EA[1]);
                   const Vector3& eb0 = pos_b(EB[0]);
                   const Vector3& eb1 = pos_b(EB[1]);

                   const Vector3& ea0_rest = rest_pos_a(EA[0]);
                   const Vector3& ea1_rest = rest_pos_a(EA[1]);
                   const Vector3& eb0_rest = rest_pos_b(EB[0]);
                   const Vector3& eb1_rest = rest_pos_b(EB[1]);

                   Vector4i flag = distance::edge_edge_distance_flag(ea0, ea1, eb0, eb1);

                   Float D;
                   distance::edge_edge_distance2(flag, ea0, ea1, eb0, eb1, D);

                   Vector12 GradD;
                   distance::edge_edge_distance2_gradient(flag, ea0, ea1, eb0, eb1, GradD);

                   Matrix12x12 HessD;
                   distance::edge_edge_distance2_hessian(flag, ea0, ea1, eb0, eb1, HessD);

                   Float eps_x;
                   distance::edge_edge_mollifier_threshold(
                       ea0_rest, ea1_rest, eb0_rest, eb1_rest, coeff, eps_x);

                   Float ek;
                   distance::edge_edge_mollifier(ea0, ea1, eb0, eb1, eps_x, ek);

                   Vector12 Gradek;
                   distance::edge_edge_mollifier_gradient(ea0, ea1, eb0, eb1, eps_x, Gradek);

                   Matrix12x12 Hessek;
                   distance::edge_edge_mollifier_hessian(ea0, ea1, eb0, eb1, eps_x, Hessek);

                   IndexT dim = distance::detail::active_count(flag);
                   if(dim < 4)
                   {
                       Gradek.setZero();
                       Hessek.setZero();
                   }

                   Vector4 coord;
                   detail::edge_edge_closest_point_coord(flag, ea0, ea1, eb0, eb1, coord);

                   // Barrier: E = e_k * B(D)
                   Float d_hat_pair = EE_d_hat(dhat_a(EA[0]), dhat_a(EA[1]),
                                               dhat_b(EB[0]), dhat_b(EB[1]));
                   Float xi = EE_thickness(thickness_a(EA[0]), thickness_a(EA[1]),
                                           thickness_b(EB[0]), thickness_b(EB[1]));
                   Vector2 dr = D_range(xi, d_hat_pair);

                   Float E = Float(0);
                   Vector12 GradE = Vector12::Zero();
                   Matrix12x12 HessE = Matrix12x12::Zero();

                   if(is_active_D(dr, D))
                   {
                       Float kappa = Float(1);
                       Float B;
                       sym::codim_ipc_contact::KappaBarrier(B, kappa, D, d_hat_pair, xi);
                       Float dBdD;
                       sym::codim_ipc_contact::dKappaBarrierdD(dBdD, kappa, D, d_hat_pair, xi);
                       Float ddBddD;
                       sym::codim_ipc_contact::ddKappaBarrierddD(ddBddD, kappa, D, d_hat_pair, xi);

                       E = ek * B;
                       GradE = ek * dBdD * GradD + B * Gradek;
                       HessE = ek * (ddBddD * GradD * GradD.transpose() + dBdD * HessD)
                             + dBdD * (GradD * Gradek.transpose() + Gradek * GradD.transpose())
                             + B * Hessek;
                   }

                   dist2(i)        = D;
                   dist2_grad(i)   = GradD;
                   dist2_hess(i)   = HessD;
                   flags(i)        = flag;
                   coords(i)       = coord;
                   eps_xs(i)       = eps_x;
                   e_ks(i)         = ek;
                   ek_grad(i)      = Gradek;
                   ek_hess(i)      = Hessek;
                   barrier(i)      = E;
                   barrier_grad(i) = GradE;
                   barrier_hess(i) = HessE;
               });

    R.instances().resize(num_pairs);

    detail::copy_back(R, "dist2",        Float(0),            dist2.view());
    detail::copy_back(R, "dist2/grad",   Vector12::Zero(),    dist2_grad.view());
    detail::copy_back(R, "dist2/hess",   Matrix12x12::Zero(), dist2_hess.view());
    detail::copy_back(R, "flag",         Vector4i::Zero(),    flags.view());
    detail::copy_back(R, "coord",        Vector4::Zero(),     coords.view());
    detail::copy_back(R, "eps_x",        Float(0),            eps_xs.view());
    detail::copy_back(R, "e_k",          Float(1),            e_ks.view());
    detail::copy_back(R, "e_k/grad",     Vector12::Zero(),    ek_grad.view());
    detail::copy_back(R, "e_k/hess",     Matrix12x12::Zero(), ek_hess.view());
    detail::copy_back(R, "barrier",      Float(0),            barrier.view());
    detail::copy_back(R, "barrier/grad", Vector12::Zero(),    barrier_grad.view());
    detail::copy_back(R, "barrier/hess", Matrix12x12::Zero(), barrier_hess.view());

    if(!mc_attr)
        R.instances().create<Float>("mollifier_coeff", mollifier_coeff_val);
}

// =====================================================================
// Point-Edge Distance (+ Barrier)
// =====================================================================
void DistanceDiagnoser::compute_point_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& edges)
{
    using namespace muda;

    auto h_point_pos = points.positions().view();
    auto h_edge_pos  = edges.positions().view();
    auto h_edge_topo = edges.edges().topo().view();

    SizeT num_points = h_point_pos.size();
    SizeT num_edges  = h_edge_topo.size();
    SizeT num_pairs  = num_points * num_edges;

    if(num_pairs == 0)
    {
        R.instances().resize(0);
        return;
    }

    DeviceBuffer<Vector3>  point_pos(num_points);
    DeviceBuffer<Vector3>  edge_pos(h_edge_pos.size());
    DeviceBuffer<Vector2i> edge_topo(num_edges);
    DeviceBuffer<Float>    dhat_p;
    DeviceBuffer<Float>    dhat_e;
    DeviceBuffer<Float>    thickness_p;
    DeviceBuffer<Float>    thickness_e;

    point_pos.view().copy_from(h_point_pos.data());
    edge_pos.view().copy_from(h_edge_pos.data());
    edge_topo.view().copy_from(h_edge_topo.data());
    detail::upload_vertex_attribute(dhat_p, points, "d_hat");
    detail::upload_vertex_attribute(dhat_e, edges,  "d_hat");
    detail::upload_vertex_attribute(thickness_p,   points, "thickness");
    detail::upload_vertex_attribute(thickness_e,   edges,  "thickness");

    DeviceBuffer<Float>     dist2(num_pairs);
    DeviceBuffer<Vector9>   dist2_grad(num_pairs);
    DeviceBuffer<Matrix9x9> dist2_hess(num_pairs);
    DeviceBuffer<Vector3i>  flags(num_pairs);
    DeviceBuffer<Vector3>   coords(num_pairs);
    DeviceBuffer<Float>     barrier(num_pairs);
    DeviceBuffer<Vector9>   barrier_grad(num_pairs);
    DeviceBuffer<Matrix9x9> barrier_hess(num_pairs);

    SizeT M = num_edges;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [point_pos    = point_pos.viewer().name("point_pos"),
                edge_pos     = edge_pos.viewer().name("edge_pos"),
                edge_topo    = edge_topo.viewer().name("edge_topo"),
                dhat_p       = dhat_p.cviewer().name("dhat_p"),
                dhat_e       = dhat_e.cviewer().name("dhat_e"),
                thickness_p         = thickness_p.cviewer().name("thickness_p"),
                thickness_e         = thickness_e.cviewer().name("thickness_e"),
                dist2        = dist2.viewer().name("dist2"),
                dist2_grad   = dist2_grad.viewer().name("dist2_grad"),
                dist2_hess   = dist2_hess.viewer().name("dist2_hess"),
                flags        = flags.viewer().name("flags"),
                coords       = coords.viewer().name("coords"),
                barrier      = barrier.viewer().name("barrier"),
                barrier_grad = barrier_grad.viewer().name("barrier_grad"),
                barrier_hess = barrier_hess.viewer().name("barrier_hess"),
                M] __device__(int i) mutable
               {
                   IndexT pi = i / M;
                   IndexT ei = i % M;

                   const Vector3& P = point_pos(pi);
                   Vector2i EI = edge_topo(ei);
                   const Vector3& E0 = edge_pos(EI[0]);
                   const Vector3& E1 = edge_pos(EI[1]);

                   Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                   Float D;
                   distance::point_edge_distance2(flag, P, E0, E1, D);

                   Vector9 GradD;
                   distance::point_edge_distance2_gradient(flag, P, E0, E1, GradD);

                   Matrix9x9 HessD;
                   distance::point_edge_distance2_hessian(flag, P, E0, E1, HessD);

                   Vector3 coord;
                   detail::point_edge_closest_point_coord(flag, P, E0, E1, coord);

                   Float d_hat_pair = PE_d_hat(dhat_p(pi), dhat_e(EI[0]), dhat_e(EI[1]));
                   Float xi         = PE_thickness(thickness_p(pi), thickness_e(EI[0]), thickness_e(EI[1]));
                   Vector2 dr = D_range(xi, d_hat_pair);

                   Float B = Float(0);
                   Vector9 GradB = Vector9::Zero();
                   Matrix9x9 HessB = Matrix9x9::Zero();

                   if(is_active_D(dr, D))
                   {
                       Float kappa = Float(1);
                       sym::codim_ipc_contact::KappaBarrier(B, kappa, D, d_hat_pair, xi);
                       Float dBdD;
                       sym::codim_ipc_contact::dKappaBarrierdD(dBdD, kappa, D, d_hat_pair, xi);
                       Float ddBddD;
                       sym::codim_ipc_contact::ddKappaBarrierddD(ddBddD, kappa, D, d_hat_pair, xi);
                       GradB = dBdD * GradD;
                       HessB = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
                   }

                   dist2(i)        = D;
                   dist2_grad(i)   = GradD;
                   dist2_hess(i)   = HessD;
                   flags(i)        = flag;
                   coords(i)       = coord;
                   barrier(i)      = B;
                   barrier_grad(i) = GradB;
                   barrier_hess(i) = HessB;
               });

    R.instances().resize(num_pairs);

    detail::copy_back(R, "dist2",        Float(0),          dist2.view());
    detail::copy_back(R, "dist2/grad",   Vector9::Zero(),   dist2_grad.view());
    detail::copy_back(R, "dist2/hess",   Matrix9x9::Zero(), dist2_hess.view());
    detail::copy_back(R, "flag",         Vector3i::Zero(),  flags.view());
    detail::copy_back(R, "coord",        Vector3::Zero(),   coords.view());
    detail::copy_back(R, "barrier",      Float(0),          barrier.view());
    detail::copy_back(R, "barrier/grad", Vector9::Zero(),   barrier_grad.view());
    detail::copy_back(R, "barrier/hess", Matrix9x9::Zero(), barrier_hess.view());
}

// =====================================================================
// Point-Point Distance (+ Barrier)
// =====================================================================
void DistanceDiagnoser::compute_point_point_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points_a,
    const geometry::SimplicialComplex& points_b)
{
    using namespace muda;

    auto h_pos_a = points_a.positions().view();
    auto h_pos_b = points_b.positions().view();

    SizeT num_a = h_pos_a.size();
    SizeT num_b = h_pos_b.size();
    SizeT num_pairs = num_a * num_b;

    if(num_pairs == 0)
    {
        R.instances().resize(0);
        return;
    }

    DeviceBuffer<Vector3> pos_a(num_a);
    DeviceBuffer<Vector3> pos_b(num_b);
    DeviceBuffer<Float>   dhat_a;
    DeviceBuffer<Float>   dhat_b;
    DeviceBuffer<Float>   thickness_a;
    DeviceBuffer<Float>   thickness_b;

    pos_a.view().copy_from(h_pos_a.data());
    pos_b.view().copy_from(h_pos_b.data());
    detail::upload_vertex_attribute(dhat_a, points_a, "d_hat");
    detail::upload_vertex_attribute(dhat_b, points_b, "d_hat");
    detail::upload_vertex_attribute(thickness_a,   points_a, "thickness");
    detail::upload_vertex_attribute(thickness_b,   points_b, "thickness");

    DeviceBuffer<Float>     dist2(num_pairs);
    DeviceBuffer<Vector6>   dist2_grad(num_pairs);
    DeviceBuffer<Matrix6x6> dist2_hess(num_pairs);
    DeviceBuffer<Float>     barrier(num_pairs);
    DeviceBuffer<Vector6>   barrier_grad(num_pairs);
    DeviceBuffer<Matrix6x6> barrier_hess(num_pairs);

    SizeT M = num_b;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [pos_a        = pos_a.viewer().name("pos_a"),
                pos_b        = pos_b.viewer().name("pos_b"),
                dhat_a       = dhat_a.cviewer().name("dhat_a"),
                dhat_b       = dhat_b.cviewer().name("dhat_b"),
                thickness_a         = thickness_a.cviewer().name("thickness_a"),
                thickness_b         = thickness_b.cviewer().name("thickness_b"),
                dist2        = dist2.viewer().name("dist2"),
                dist2_grad   = dist2_grad.viewer().name("dist2_grad"),
                dist2_hess   = dist2_hess.viewer().name("dist2_hess"),
                barrier      = barrier.viewer().name("barrier"),
                barrier_grad = barrier_grad.viewer().name("barrier_grad"),
                barrier_hess = barrier_hess.viewer().name("barrier_hess"),
                M] __device__(int i) mutable
               {
                   IndexT ai = i / M;
                   IndexT bi = i % M;

                   const Vector3& A = pos_a(ai);
                   const Vector3& B = pos_b(bi);

                   Float D;
                   distance::point_point_distance2(A, B, D);

                   Vector6 GradD;
                   distance::point_point_distance2_gradient(A, B, GradD);

                   Matrix6x6 HessD;
                   distance::point_point_distance2_hessian(A, B, HessD);

                   Float d_hat_pair = PP_d_hat(dhat_a(ai), dhat_b(bi));
                   Float xi         = PP_thickness(thickness_a(ai), thickness_b(bi));
                   Vector2 dr = D_range(xi, d_hat_pair);

                   Float Bv = Float(0);
                   Vector6 GradB = Vector6::Zero();
                   Matrix6x6 HessB = Matrix6x6::Zero();

                   if(is_active_D(dr, D))
                   {
                       Float kappa = Float(1);
                       sym::codim_ipc_contact::KappaBarrier(Bv, kappa, D, d_hat_pair, xi);
                       Float dBdD;
                       sym::codim_ipc_contact::dKappaBarrierdD(dBdD, kappa, D, d_hat_pair, xi);
                       Float ddBddD;
                       sym::codim_ipc_contact::ddKappaBarrierddD(ddBddD, kappa, D, d_hat_pair, xi);
                       GradB = dBdD * GradD;
                       HessB = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
                   }

                   dist2(i)        = D;
                   dist2_grad(i)   = GradD;
                   dist2_hess(i)   = HessD;
                   barrier(i)      = Bv;
                   barrier_grad(i) = GradB;
                   barrier_hess(i) = HessB;
               });

    R.instances().resize(num_pairs);

    detail::copy_back(R, "dist2",        Float(0),          dist2.view());
    detail::copy_back(R, "dist2/grad",   Vector6::Zero(),   dist2_grad.view());
    detail::copy_back(R, "dist2/hess",   Matrix6x6::Zero(), dist2_hess.view());
    detail::copy_back(R, "barrier",      Float(0),          barrier.view());
    detail::copy_back(R, "barrier/grad", Vector6::Zero(),   barrier_grad.view());
    detail::copy_back(R, "barrier/hess", Matrix6x6::Zero(), barrier_hess.view());
}
}  // namespace uipc::backend::cuda
