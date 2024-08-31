//ref: https://github.com/ipc-sim/Codim-IPC/tree/main/Library/Math/Distance
#include <thrust/extrema.h>
#include <thrust/swap.h>
namespace uipc::backend::cuda::distance
{
template <typename T>
MUDA_GENERIC bool point_edge_cd_broadphase(const Eigen::Vector<T, 3>& x0,
                                           const Eigen::Vector<T, 3>& x1,
                                           const Eigen::Vector<T, 3>& x2,
                                           T                          dist)
{
    const Eigen::Array<T, 3, 1> max_e = x1.array().max(x2.array());
    const Eigen::Array<T, 3, 1> min_e = x1.array().min(x2.array());
    if((x0.array() - max_e > dist).any() || (min_e - x0.array() > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Matrix<T, 2, 1>& p,
                                            const Eigen::Matrix<T, 2, 1>& e0,
                                            const Eigen::Matrix<T, 2, 1>& e1,
                                            const Eigen::Matrix<T, 2, 1>& dp,
                                            const Eigen::Matrix<T, 2, 1>& de0,
                                            const Eigen::Matrix<T, 2, 1>& de1,
                                            T                             dist)
{
    const Eigen::Array<T, 2, 1> max_p = p.array().max((p + dp).array());
    const Eigen::Array<T, 2, 1> min_p = p.array().min((p + dp).array());
    const Eigen::Array<T, 2, 1> max_e =
        e0.array().max(e1.array()).max((e0 + de0).array()).max((e1 + de1).array());
    const Eigen::Array<T, 2, 1> min_e =
        e0.array().min(e1.array()).min((e0 + de0).array()).min((e1 + de1).array());
    if((min_p - max_e > dist).any() || (min_e - max_p > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_triangle_cd_broadphase(const Eigen::Vector<T, 3>& p,
                                               const Eigen::Vector<T, 3>& t0,
                                               const Eigen::Vector<T, 3>& t1,
                                               const Eigen::Vector<T, 3>& t2,
                                               T                          dist)
{
    const Eigen::Array<T, 3, 1> max_tri = t0.array().max(t1.array()).max(t2.array());
    const Eigen::Array<T, 3, 1> min_tri = t0.array().min(t1.array()).min(t2.array());
    if((p.array() - max_tri > dist).any() || (min_tri - p.array() > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool edge_edge_cd_broadphase(const Eigen::Vector<T, 3>& ea0,
                                          const Eigen::Vector<T, 3>& ea1,
                                          const Eigen::Vector<T, 3>& eb0,
                                          const Eigen::Vector<T, 3>& eb1,
                                          T                          dist)
{
    const Eigen::Array<T, 3, 1> max_a = ea0.array().max(ea1.array());
    const Eigen::Array<T, 3, 1> min_a = ea0.array().min(ea1.array());
    const Eigen::Array<T, 3, 1> max_b = eb0.array().max(eb1.array());
    const Eigen::Array<T, 3, 1> min_b = eb0.array().min(eb1.array());
    if((min_a - max_b > dist).any() || (min_b - max_a > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_triangle_ccd_broadphase(const Eigen::Vector<T, 3>& p,
                                                const Eigen::Vector<T, 3>& t0,
                                                const Eigen::Vector<T, 3>& t1,
                                                const Eigen::Vector<T, 3>& t2,
                                                const Eigen::Vector<T, 3>& dp,
                                                const Eigen::Vector<T, 3>& dt0,
                                                const Eigen::Vector<T, 3>& dt1,
                                                const Eigen::Vector<T, 3>& dt2,
                                                T                          dist)
{
    const Eigen::Array<T, 3, 1> max_p   = p.array().max((p + dp).array());
    const Eigen::Array<T, 3, 1> min_p   = p.array().min((p + dp).array());
    const Eigen::Array<T, 3, 1> max_tri = t0.array()
                                              .max(t1.array())
                                              .max(t2.array())
                                              .max((t0 + dt0).array())
                                              .max((t1 + dt1).array())
                                              .max((t2 + dt2).array());
    const Eigen::Array<T, 3, 1> min_tri = t0.array()
                                              .min(t1.array())
                                              .min(t2.array())
                                              .min((t0 + dt0).array())
                                              .min((t1 + dt1).array())
                                              .min((t2 + dt2).array());
    if((min_p - max_tri > dist).any() || (min_tri - max_p > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool edge_edge_ccd_broadphase(const Eigen::Vector<T, 3>& ea0,
                                           const Eigen::Vector<T, 3>& ea1,
                                           const Eigen::Vector<T, 3>& eb0,
                                           const Eigen::Vector<T, 3>& eb1,
                                           const Eigen::Vector<T, 3>& dea0,
                                           const Eigen::Vector<T, 3>& dea1,
                                           const Eigen::Vector<T, 3>& deb0,
                                           const Eigen::Vector<T, 3>& deb1,
                                           T                          dist)
{
    const Eigen::Array<T, 3, 1> max_a =
        ea0.array().max(ea1.array()).max((ea0 + dea0).array()).max((ea1 + dea1).array());
    const Eigen::Array<T, 3, 1> min_a =
        ea0.array().min(ea1.array()).min((ea0 + dea0).array()).min((ea1 + dea1).array());
    const Eigen::Array<T, 3, 1> max_b =
        eb0.array().max(eb1.array()).max((eb0 + deb0).array()).max((eb1 + deb1).array());
    const Eigen::Array<T, 3, 1> min_b =
        eb0.array().min(eb1.array()).min((eb0 + deb0).array()).min((eb1 + deb1).array());
    if((min_a - max_b > dist).any() || (min_b - max_a > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Vector<T, 3>& p,
                                            const Eigen::Vector<T, 3>& e0,
                                            const Eigen::Vector<T, 3>& e1,
                                            const Eigen::Vector<T, 3>& dp,
                                            const Eigen::Vector<T, 3>& de0,
                                            const Eigen::Vector<T, 3>& de1,
                                            T                          dist)
{
    const Eigen::Array<T, 3, 1> max_p = p.array().max((p + dp).array());
    const Eigen::Array<T, 3, 1> min_p = p.array().min((p + dp).array());
    const Eigen::Array<T, 3, 1> max_e =
        e0.array().max(e1.array()).max((e0 + de0).array()).max((e1 + de1).array());
    const Eigen::Array<T, 3, 1> min_e =
        e0.array().min(e1.array()).min((e0 + de0).array()).min((e1 + de1).array());
    if((min_p - max_e > dist).any() || (min_e - max_p > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_point_ccd_broadphase(const Eigen::Vector<T, 3>& p0,
                                             const Eigen::Vector<T, 3>& p1,
                                             const Eigen::Vector<T, 3>& dp0,
                                             const Eigen::Vector<T, 3>& dp1,
                                             T                          dist)
{
    const Eigen::Array<T, 3, 1> max_p0 = p0.array().max((p0 + dp0).array());
    const Eigen::Array<T, 3, 1> min_p0 = p0.array().min((p0 + dp0).array());
    const Eigen::Array<T, 3, 1> max_p1 = p1.array().max((p1 + dp1).array());
    const Eigen::Array<T, 3, 1> min_p1 = p1.array().min((p1 + dp1).array());
    if((min_p0 - max_p1 > dist).any() || (min_p1 - max_p0 > dist).any())
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <typename T>
MUDA_GENERIC bool point_triangle_ccd(Eigen::Vector<T, 3> p,
                                     Eigen::Vector<T, 3> t0,
                                     Eigen::Vector<T, 3> t1,
                                     Eigen::Vector<T, 3> t2,
                                     Eigen::Vector<T, 3> dp,
                                     Eigen::Vector<T, 3> dt0,
                                     Eigen::Vector<T, 3> dt1,
                                     Eigen::Vector<T, 3> dt2,
                                     T                   eta,
                                     T                   thickness,
                                     int                 max_iter,
                                     T&                  toc)
{
    Eigen::Vector<T, 3> mov = (dt0 + dt1 + dt2 + dp) / 4;
    dt0 -= mov;
    dt1 -= mov;
    dt2 -= mov;
    dp -= mov;
    Eigen::Array3<T> dispMag2Vec{dt0.squaredNorm(), dt1.squaredNorm(), dt2.squaredNorm()};
    T maxDispMag = dp.norm() + sqrt(dispMag2Vec.maxCoeff());

    if(maxDispMag <= T(0))
    {
        return false;
    }

    T    dist2_cur;
    auto flag = point_triangle_distance_flag(p, t0, t1, t2);
    point_triangle_distance2(flag, p, t0, t1, t2, dist2_cur);
    T dist_cur = sqrt(dist2_cur);
    T gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    T toc_prev = toc;
    toc        = 0;
    while(true)
    {
        if(max_iter >= 0)
        {
            if(--max_iter < 0)
                return true;
        }

        T tocLowerBound = (1 - eta) * (dist2_cur - thickness * thickness)
                          / ((dist_cur + thickness) * maxDispMag);

        p += tocLowerBound * dp;
        t0 += tocLowerBound * dt0;
        t1 += tocLowerBound * dt1;
        t2 += tocLowerBound * dt2;
        flag = point_triangle_distance_flag(p, t0, t1, t2);
        point_triangle_distance2(flag, p, t0, t1, t2, dist2_cur);
        dist_cur = sqrt(dist2_cur);
        if(toc && ((dist2_cur - thickness * thickness) / (dist_cur + thickness) < gap))
        {
            break;
        }

        toc += tocLowerBound;
        if(toc > toc_prev)
        {
            return false;
        }
    }

    return true;
}

template <typename T>
MUDA_GENERIC bool edge_edge_ccd(Eigen::Vector<T, 3> ea0,
                                Eigen::Vector<T, 3> ea1,
                                Eigen::Vector<T, 3> eb0,
                                Eigen::Vector<T, 3> eb1,
                                Eigen::Vector<T, 3> dea0,
                                Eigen::Vector<T, 3> dea1,
                                Eigen::Vector<T, 3> deb0,
                                Eigen::Vector<T, 3> deb1,
                                T                   eta,
                                T                   thickness,
                                int                 max_iter,
                                T&                  toc)
{
    Eigen::Vector<T, 3> mov = (dea0 + dea1 + deb0 + deb1) / 4;
    dea0 -= mov;
    dea1 -= mov;
    deb0 -= mov;
    deb1 -= mov;
    T maxDispMag = sqrt(std::max(dea0.squaredNorm(), dea1.squaredNorm()))
                   + sqrt(std::max(deb0.squaredNorm(), deb1.squaredNorm()));
    if(maxDispMag == 0)
    {
        return false;
    }

    T    dist2_cur;
    auto flag = edge_edge_distance_flag(ea0, ea1, eb0, eb1);
    edge_edge_distance2(flag, ea0, ea1, eb0, eb1, dist2_cur);
    T dFunc = dist2_cur - thickness * thickness;
    if(dFunc <= 0)
    {
        // since we ensured other place that all dist smaller than dHat are positive,
        // this must be some far away nearly parallel edges
        Eigen::Array4<T> dists{(ea0 - eb0).squaredNorm(),
                               (ea0 - eb1).squaredNorm(),
                               (ea1 - eb0).squaredNorm(),
                               (ea1 - eb1).squaredNorm()};
        dist2_cur = dists.minCoeff();
        dFunc     = dist2_cur - thickness * thickness;
    }
    T dist_cur = sqrt(dist2_cur);
    T gap      = eta * dFunc / (dist_cur + thickness);
    T toc_prev = toc;
    toc        = 0;
    while(true)
    {
        if(max_iter >= 0)
        {
            if(--max_iter < 0)
                return true;
        }

        T tocLowerBound = (1 - eta) * dFunc / ((dist_cur + thickness) * maxDispMag);

        ea0 += tocLowerBound * dea0;
        ea1 += tocLowerBound * dea1;
        eb0 += tocLowerBound * deb0;
        eb1 += tocLowerBound * deb1;
        auto flag = edge_edge_distance_flag(ea0, ea1, eb0, eb1);
        edge_edge_distance2(flag, ea0, ea1, eb0, eb1, dist2_cur);
        dFunc = dist2_cur - thickness * thickness;
        if(dFunc <= 0)
        {
            // since we ensured other place that all dist smaller than dHat are positive,
            // this must be some far away nearly parallel edges
            Eigen::Array4<T> dists{(ea0 - eb0).squaredNorm(),
                                   (ea0 - eb1).squaredNorm(),
                                   (ea1 - eb0).squaredNorm(),
                                   (ea1 - eb1).squaredNorm()};
            dist2_cur = dists.minCoeff();
            dFunc     = dist2_cur - thickness * thickness;
        }
        dist_cur = sqrt(dist2_cur);
        if(toc && (dFunc / (dist_cur + thickness) < gap))
        {
            break;
        }

        toc += tocLowerBound;
        if(toc > toc_prev)
        {
            return false;
        }
    }

    return true;
}

template <typename T>
MUDA_GENERIC bool point_edge_ccd(Eigen::Vector<T, 3> p,
                                 Eigen::Vector<T, 3> e0,
                                 Eigen::Vector<T, 3> e1,
                                 Eigen::Vector<T, 3> dp,
                                 Eigen::Vector<T, 3> de0,
                                 Eigen::Vector<T, 3> de1,
                                 T                   eta,
                                 T                   thickness,
                                 int                 max_iter,
                                 T&                  toc)
{
    Eigen::Vector<T, 3> mov = (dp + de0 + de1) / 3;
    de0 -= mov;
    de1 -= mov;
    dp -= mov;
    T maxDispMag = dp.norm() + sqrt(std::max(de0.squaredNorm(), de1.squaredNorm()));
    if(maxDispMag == 0)
    {
        return false;
    }

    T    dist2_cur;
    auto flag = point_edge_distance_flag(p, e0, e1);
    point_edge_distance2(flag, p, e0, e1, dist2_cur);
    T dist_cur = sqrt(dist2_cur);
    T gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    T toc_prev = toc;
    toc        = 0;
    while(true)
    {
        if(max_iter >= 0)
        {
            if(--max_iter < 0)
                return true;
        }

        T tocLowerBound = (1 - eta) * (dist2_cur - thickness * thickness)
                          / ((dist_cur + thickness) * maxDispMag);

        p += tocLowerBound * dp;
        e0 += tocLowerBound * de0;
        e1 += tocLowerBound * de1;
        flag = point_edge_distance_flag(p, e0, e1);
        point_edge_distance2(flag, p, e0, e1, dist2_cur);
        dist_cur = sqrt(dist2_cur);
        if(toc && (dist2_cur - thickness * thickness) / (dist_cur + thickness) < gap)
        {
            break;
        }

        toc += tocLowerBound;
        if(toc > toc_prev)
        {
            return false;
        }
    }

    return true;
}

template <typename T>
MUDA_GENERIC bool point_point_ccd(Eigen::Vector<T, 3> p0,
                                  Eigen::Vector<T, 3> p1,
                                  Eigen::Vector<T, 3> dp0,
                                  Eigen::Vector<T, 3> dp1,
                                  T                   eta,
                                  T                   thickness,
                                  int                 max_iter,
                                  T&                  toc)
{
    Eigen::Vector<T, 3> mov = (dp0 + dp1) / 2;
    dp1 -= mov;
    dp0 -= mov;
    T maxDispMag = dp0.norm() + dp1.norm();
    if(maxDispMag == 0)
    {
        return false;
    }

    T    dist2_cur;
    auto flag = point_point_distance_flag(p0, p1);
    point_point_distance2(flag, p0, p1, dist2_cur);
    T dist_cur = sqrt(dist2_cur);
    T gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    T toc_prev = toc;
    toc        = 0;
    while(true)
    {
        if(max_iter >= 0)
        {
            if(--max_iter < 0)
                return true;
        }

        T tocLowerBound = (1 - eta) * (dist2_cur - thickness * thickness)
                          / ((dist_cur + thickness) * maxDispMag);

        p0 += tocLowerBound * dp0;
        p1 += tocLowerBound * dp1;
        flag = point_point_distance_flag(p0, p1);
        point_point_distance2(flag, p0, p1, dist2_cur);
        dist_cur = sqrt(dist2_cur);
        if(toc && (dist2_cur - thickness * thickness) / (dist_cur + thickness) < gap)
        {
            break;
        }

        toc += tocLowerBound;
        if(toc > toc_prev)
        {
            return false;
        }
    }

    return true;
}
}  // namespace uipc::backend::cuda::distance