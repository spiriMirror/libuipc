#include <distance_system/distance_diagnoser.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance/edge_edge_mollifier.h>
#include <muda/buffer/device_buffer.h>
#include <muda/launch/parallel_for.h>
#include <uipc/common/enumerate.h>

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
    // Compute barycentric coordinates of the closest point on the triangle
    // in the {p, t0, t1, t2} frame. coord[0] is always 0.
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
            // PP: closest to a single triangle vertex
            if(flag[1]) coord[1] = T(1);
            else if(flag[2]) coord[2] = T(1);
            else if(flag[3]) coord[3] = T(1);
        }
        else if(dim == 3)
        {
            // PE: closest to an edge of the triangle
            // Find which two triangle vertices are active
            Eigen::Vector<T, 3> e0, e1;
            int idx0 = -1, idx1 = -1;
            if(flag[1] && flag[2])      { e0 = t0; e1 = t1; idx0 = 1; idx1 = 2; }
            else if(flag[2] && flag[3]) { e0 = t1; e1 = t2; idx0 = 2; idx1 = 3; }
            else if(flag[3] && flag[1]) { e0 = t2; e1 = t0; idx0 = 3; idx1 = 1; }

            // Project p onto the edge e0-e1
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
            // PT: closest point is on the triangle interior (projection)
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

            coord[1] = u;  // weight for t0
            coord[2] = v;  // weight for t1
            coord[3] = w;  // weight for t2
        }
    }
}  // namespace detail

void DistanceDiagnoser::compute_point_triangle_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& triangles)
{
    using namespace muda;

    // ------------------------------------------------------------------
    // 1. Read host geometry
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // 2. Upload to device
    // ------------------------------------------------------------------
    DeviceBuffer<Vector3>  point_pos(num_points);
    DeviceBuffer<Vector3>  tri_pos(h_tri_pos.size());
    DeviceBuffer<Vector3i> tri_topo(num_triangles);

    point_pos.view().copy_from(h_point_pos.data());
    tri_pos.view().copy_from(h_tri_pos.data());
    tri_topo.view().copy_from(h_tri_topo.data());

    // ------------------------------------------------------------------
    // 3. Allocate device result buffers
    // ------------------------------------------------------------------
    DeviceBuffer<Float>      dist2_buf(num_pairs);
    DeviceBuffer<Vector12>   grad_buf(num_pairs);
    DeviceBuffer<Matrix12x12> hess_buf(num_pairs);
    DeviceBuffer<Vector4i>   flag_buf(num_pairs);
    DeviceBuffer<Vector4>    coord_buf(num_pairs);

    // ------------------------------------------------------------------
    // 4. Launch kernel
    // ------------------------------------------------------------------
    SizeT M = num_triangles;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [point_pos = point_pos.viewer().name("point_pos"),
                tri_pos   = tri_pos.viewer().name("tri_pos"),
                tri_topo  = tri_topo.viewer().name("tri_topo"),
                dist2_out = dist2_buf.viewer().name("dist2"),
                grad_out  = grad_buf.viewer().name("grad"),
                hess_out  = hess_buf.viewer().name("hess"),
                flag_out  = flag_buf.viewer().name("flag"),
                coord_out = coord_buf.viewer().name("coord"),
                M] __device__(int idx) mutable
               {
                   IndexT pi = idx / M;
                   IndexT ti = idx % M;

                   const Vector3& P  = point_pos(pi);
                   Vector3i       TI = tri_topo(ti);
                   const Vector3& T0 = tri_pos(TI[0]);
                   const Vector3& T1 = tri_pos(TI[1]);
                   const Vector3& T2 = tri_pos(TI[2]);

                   Vector4i flag = distance::point_triangle_distance_flag(P, T0, T1, T2);

                   Float D;
                   distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

                   Vector12 G;
                   distance::point_triangle_distance2_gradient(flag, P, T0, T1, T2, G);

                   Matrix12x12 H;
                   distance::point_triangle_distance2_hessian(flag, P, T0, T1, T2, H);

                   Vector4 coord;
                   detail::point_triangle_closest_point_coord(flag, P, T0, T1, T2, coord);

                   dist2_out(idx) = D;
                   grad_out(idx)  = G;
                   hess_out(idx)  = H;
                   flag_out(idx)  = flag;
                   coord_out(idx) = coord;
               });

    // ------------------------------------------------------------------
    // 5. Copy results back to host geometry R
    // ------------------------------------------------------------------
    R.instances().resize(num_pairs);

    auto dist2_attr = R.instances().find<Float>("dist2");
    if(!dist2_attr)
        dist2_attr = R.instances().create<Float>("dist2", 0.0);
    dist2_buf.view().copy_to(view(*dist2_attr).data());

    auto grad_attr = R.instances().find<Vector12>("grad");
    if(!grad_attr)
        grad_attr = R.instances().create<Vector12>("grad", Vector12::Zero());
    grad_buf.view().copy_to(view(*grad_attr).data());

    auto hess_attr = R.instances().find<Matrix12x12>("hess");
    if(!hess_attr)
        hess_attr = R.instances().create<Matrix12x12>("hess", Matrix12x12::Zero());
    hess_buf.view().copy_to(view(*hess_attr).data());

    auto flag_attr = R.instances().find<Vector4i>("flag");
    if(!flag_attr)
        flag_attr = R.instances().create<Vector4i>("flag", Vector4i::Zero());
    flag_buf.view().copy_to(view(*flag_attr).data());

    auto coord_attr = R.instances().find<Vector4>("coord");
    if(!coord_attr)
        coord_attr = R.instances().create<Vector4>("coord", Vector4::Zero());
    coord_buf.view().copy_to(view(*coord_attr).data());
}

// =====================================================================
// Edge-Edge helpers
// =====================================================================
namespace detail
{
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
            // PP: two specific vertices are closest
            if(flag[0]) coord[0] = T(1);
            else        coord[1] = T(1);

            if(flag[2]) coord[2] = T(1);
            else        coord[3] = T(1);
        }
        else if(dim == 3)
        {
            // PE: one vertex closest to the other edge
            Eigen::Vector<T, 3> point, e0, e1;
            int pt_idx = -1, e0_idx = -1, e1_idx = -1;

            if(!flag[0])
            {
                // ea1 is the point, eb0-eb1 is the edge
                point = ea1; pt_idx = 1;
                e0 = eb0; e0_idx = 2;
                e1 = eb1; e1_idx = 3;
            }
            else if(!flag[1])
            {
                // ea0 is the point, eb0-eb1 is the edge
                point = ea0; pt_idx = 0;
                e0 = eb0; e0_idx = 2;
                e1 = eb1; e1_idx = 3;
            }
            else if(!flag[2])
            {
                // eb1 is the point, ea0-ea1 is the edge
                point = eb1; pt_idx = 3;
                e0 = ea0; e0_idx = 0;
                e1 = ea1; e1_idx = 1;
            }
            else
            {
                // eb0 is the point, ea0-ea1 is the edge
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
            // Full EE: line-line closest points
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

            coord[0] = T(1) - s;  // w_ea0
            coord[1] = s;          // w_ea1
            coord[2] = T(1) - t;  // w_eb0
            coord[3] = t;          // w_eb1
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
            // PP: closest to an endpoint
            if(flag[1]) coord[1] = T(1);
            else        coord[2] = T(1);
        }
        else if(dim == 3)
        {
            // Full PE: project onto edge interior
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
}  // namespace detail

// =====================================================================
// Edge-Edge Distance (with Mollifier)
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

    // Read mollifier_coeff from R (or create with default)
    auto mc_attr = R.instances().find<Float>("mollifier_coeff");
    Float mollifier_coeff_val = Float(1e-3);
    if(mc_attr)
    {
        auto mc_view = mc_attr->view();
        if(mc_view.size() > 0)
            mollifier_coeff_val = mc_view[0];
    }

    // Upload to device
    DeviceBuffer<Vector3>  pos_a(h_pos_a.size());
    DeviceBuffer<Vector3>  pos_b(h_pos_b.size());
    DeviceBuffer<Vector2i> topo_a(num_ea);
    DeviceBuffer<Vector2i> topo_b(num_eb);
    DeviceBuffer<Vector3>  rest_pos_a(h_rest_pos_a.size());
    DeviceBuffer<Vector3>  rest_pos_b(h_rest_pos_b.size());

    pos_a.view().copy_from(h_pos_a.data());
    pos_b.view().copy_from(h_pos_b.data());
    topo_a.view().copy_from(h_topo_a.data());
    topo_b.view().copy_from(h_topo_b.data());
    rest_pos_a.view().copy_from(h_rest_pos_a.data());
    rest_pos_b.view().copy_from(h_rest_pos_b.data());

    // Allocate result buffers
    DeviceBuffer<Float>        dist2_buf(num_pairs);
    DeviceBuffer<Vector12>     grad_buf(num_pairs);
    DeviceBuffer<Matrix12x12>  hess_buf(num_pairs);
    DeviceBuffer<Vector4i>     flag_buf(num_pairs);
    DeviceBuffer<Vector4>      coord_buf(num_pairs);
    DeviceBuffer<Float>        eps_x_buf(num_pairs);
    DeviceBuffer<Float>        mollifier_buf(num_pairs);
    DeviceBuffer<Vector12>     moll_grad_buf(num_pairs);
    DeviceBuffer<Matrix12x12>  moll_hess_buf(num_pairs);

    SizeT M = num_eb;
    Float coeff = mollifier_coeff_val;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [pos_a      = pos_a.viewer().name("pos_a"),
                pos_b      = pos_b.viewer().name("pos_b"),
                topo_a     = topo_a.viewer().name("topo_a"),
                topo_b     = topo_b.viewer().name("topo_b"),
                rest_pos_a = rest_pos_a.viewer().name("rest_pos_a"),
                rest_pos_b = rest_pos_b.viewer().name("rest_pos_b"),
                dist2_out      = dist2_buf.viewer().name("dist2"),
                grad_out       = grad_buf.viewer().name("grad"),
                hess_out       = hess_buf.viewer().name("hess"),
                flag_out       = flag_buf.viewer().name("flag"),
                coord_out      = coord_buf.viewer().name("coord"),
                eps_x_out      = eps_x_buf.viewer().name("eps_x"),
                moll_out       = mollifier_buf.viewer().name("mollifier"),
                moll_grad_out  = moll_grad_buf.viewer().name("mollifier_grad"),
                moll_hess_out  = moll_hess_buf.viewer().name("mollifier_hess"),
                M, coeff] __device__(int idx) mutable
               {
                   IndexT ai = idx / M;
                   IndexT bi = idx % M;

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

                   Vector4 coord;
                   detail::edge_edge_closest_point_coord(flag, ea0, ea1, eb0, eb1, coord);

                   dist2_out(idx)     = D;
                   grad_out(idx)      = GradD;
                   hess_out(idx)      = HessD;
                   flag_out(idx)      = flag;
                   coord_out(idx)     = coord;
                   eps_x_out(idx)     = eps_x;
                   moll_out(idx)      = ek;
                   moll_grad_out(idx) = Gradek;
                   moll_hess_out(idx) = Hessek;
               });

    // Copy results back
    R.instances().resize(num_pairs);

    auto dist2_attr = R.instances().find<Float>("dist2");
    if(!dist2_attr)
        dist2_attr = R.instances().create<Float>("dist2", 0.0);
    dist2_buf.view().copy_to(view(*dist2_attr).data());

    auto grad_attr = R.instances().find<Vector12>("grad");
    if(!grad_attr)
        grad_attr = R.instances().create<Vector12>("grad", Vector12::Zero());
    grad_buf.view().copy_to(view(*grad_attr).data());

    auto hess_attr = R.instances().find<Matrix12x12>("hess");
    if(!hess_attr)
        hess_attr = R.instances().create<Matrix12x12>("hess", Matrix12x12::Zero());
    hess_buf.view().copy_to(view(*hess_attr).data());

    auto flag_attr = R.instances().find<Vector4i>("flag");
    if(!flag_attr)
        flag_attr = R.instances().create<Vector4i>("flag", Vector4i::Zero());
    flag_buf.view().copy_to(view(*flag_attr).data());

    auto coord_attr = R.instances().find<Vector4>("coord");
    if(!coord_attr)
        coord_attr = R.instances().create<Vector4>("coord", Vector4::Zero());
    coord_buf.view().copy_to(view(*coord_attr).data());

    auto eps_x_attr = R.instances().find<Float>("eps_x");
    if(!eps_x_attr)
        eps_x_attr = R.instances().create<Float>("eps_x", 0.0);
    eps_x_buf.view().copy_to(view(*eps_x_attr).data());

    auto moll_attr = R.instances().find<Float>("mollifier");
    if(!moll_attr)
        moll_attr = R.instances().create<Float>("mollifier", 1.0);
    mollifier_buf.view().copy_to(view(*moll_attr).data());

    auto moll_grad_attr = R.instances().find<Vector12>("mollifier_grad");
    if(!moll_grad_attr)
        moll_grad_attr = R.instances().create<Vector12>("mollifier_grad", Vector12::Zero());
    moll_grad_buf.view().copy_to(view(*moll_grad_attr).data());

    auto moll_hess_attr = R.instances().find<Matrix12x12>("mollifier_hess");
    if(!moll_hess_attr)
        moll_hess_attr = R.instances().create<Matrix12x12>("mollifier_hess", Matrix12x12::Zero());
    moll_hess_buf.view().copy_to(view(*moll_hess_attr).data());

    if(!mc_attr)
        R.instances().create<Float>("mollifier_coeff", mollifier_coeff_val);
}

// =====================================================================
// Point-Edge Distance
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

    point_pos.view().copy_from(h_point_pos.data());
    edge_pos.view().copy_from(h_edge_pos.data());
    edge_topo.view().copy_from(h_edge_topo.data());

    DeviceBuffer<Float>     dist2_buf(num_pairs);
    DeviceBuffer<Vector9>   grad_buf(num_pairs);
    DeviceBuffer<Matrix9x9> hess_buf(num_pairs);
    DeviceBuffer<Vector3i>  flag_buf(num_pairs);
    DeviceBuffer<Vector3>   coord_buf(num_pairs);

    SizeT M = num_edges;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [point_pos = point_pos.viewer().name("point_pos"),
                edge_pos  = edge_pos.viewer().name("edge_pos"),
                edge_topo = edge_topo.viewer().name("edge_topo"),
                dist2_out = dist2_buf.viewer().name("dist2"),
                grad_out  = grad_buf.viewer().name("grad"),
                hess_out  = hess_buf.viewer().name("hess"),
                flag_out  = flag_buf.viewer().name("flag"),
                coord_out = coord_buf.viewer().name("coord"),
                M] __device__(int idx) mutable
               {
                   IndexT pi = idx / M;
                   IndexT ei = idx % M;

                   const Vector3& P = point_pos(pi);
                   Vector2i EI = edge_topo(ei);
                   const Vector3& E0 = edge_pos(EI[0]);
                   const Vector3& E1 = edge_pos(EI[1]);

                   Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                   Float D;
                   distance::point_edge_distance2(flag, P, E0, E1, D);

                   Vector9 G;
                   distance::point_edge_distance2_gradient(flag, P, E0, E1, G);

                   Matrix9x9 H;
                   distance::point_edge_distance2_hessian(flag, P, E0, E1, H);

                   Vector3 coord;
                   detail::point_edge_closest_point_coord(flag, P, E0, E1, coord);

                   dist2_out(idx) = D;
                   grad_out(idx)  = G;
                   hess_out(idx)  = H;
                   flag_out(idx)  = flag;
                   coord_out(idx) = coord;
               });

    R.instances().resize(num_pairs);

    auto dist2_attr = R.instances().find<Float>("dist2");
    if(!dist2_attr)
        dist2_attr = R.instances().create<Float>("dist2", 0.0);
    dist2_buf.view().copy_to(view(*dist2_attr).data());

    auto grad_attr = R.instances().find<Vector9>("grad");
    if(!grad_attr)
        grad_attr = R.instances().create<Vector9>("grad", Vector9::Zero());
    grad_buf.view().copy_to(view(*grad_attr).data());

    auto hess_attr = R.instances().find<Matrix9x9>("hess");
    if(!hess_attr)
        hess_attr = R.instances().create<Matrix9x9>("hess", Matrix9x9::Zero());
    hess_buf.view().copy_to(view(*hess_attr).data());

    auto flag_attr = R.instances().find<Vector3i>("flag");
    if(!flag_attr)
        flag_attr = R.instances().create<Vector3i>("flag", Vector3i::Zero());
    flag_buf.view().copy_to(view(*flag_attr).data());

    auto coord_attr = R.instances().find<Vector3>("coord");
    if(!coord_attr)
        coord_attr = R.instances().create<Vector3>("coord", Vector3::Zero());
    coord_buf.view().copy_to(view(*coord_attr).data());
}

// =====================================================================
// Point-Point Distance
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

    pos_a.view().copy_from(h_pos_a.data());
    pos_b.view().copy_from(h_pos_b.data());

    DeviceBuffer<Float>     dist2_buf(num_pairs);
    DeviceBuffer<Vector6>   grad_buf(num_pairs);
    DeviceBuffer<Matrix6x6> hess_buf(num_pairs);

    SizeT M = num_b;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_pairs,
               [pos_a     = pos_a.viewer().name("pos_a"),
                pos_b     = pos_b.viewer().name("pos_b"),
                dist2_out = dist2_buf.viewer().name("dist2"),
                grad_out  = grad_buf.viewer().name("grad"),
                hess_out  = hess_buf.viewer().name("hess"),
                M] __device__(int idx) mutable
               {
                   IndexT ai = idx / M;
                   IndexT bi = idx % M;

                   const Vector3& A = pos_a(ai);
                   const Vector3& B = pos_b(bi);

                   Float D;
                   distance::point_point_distance2(A, B, D);

                   Vector6 G;
                   distance::point_point_distance2_gradient(A, B, G);

                   Matrix6x6 H;
                   distance::point_point_distance2_hessian(A, B, H);

                   dist2_out(idx) = D;
                   grad_out(idx)  = G;
                   hess_out(idx)  = H;
               });

    R.instances().resize(num_pairs);

    auto dist2_attr = R.instances().find<Float>("dist2");
    if(!dist2_attr)
        dist2_attr = R.instances().create<Float>("dist2", 0.0);
    dist2_buf.view().copy_to(view(*dist2_attr).data());

    auto grad_attr = R.instances().find<Vector6>("grad");
    if(!grad_attr)
        grad_attr = R.instances().create<Vector6>("grad", Vector6::Zero());
    grad_buf.view().copy_to(view(*grad_attr).data());

    auto hess_attr = R.instances().find<Matrix6x6>("hess");
    if(!hess_attr)
        hess_attr = R.instances().create<Matrix6x6>("hess", Matrix6x6::Zero());
    hess_buf.view().copy_to(view(*hess_attr).data());
}
}  // namespace uipc::backend::cuda
