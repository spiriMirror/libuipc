//ref: https://github.com/ipc-sim/Codim-IPC/tree/main/Library/Math/Distance
#include <thrust/extrema.h>
#include <thrust/swap.h>
namespace uipc::backend::cuda::distance
{
template <typename T, int dim>
MUDA_GENERIC bool point_edge_cd_broadphase(const Eigen::Matrix<T, dim, 1>& x0,
                                           const Eigen::Matrix<T, dim, 1>& x1,
                                           const Eigen::Matrix<T, dim, 1>& x2,
                                           T                               dist)
{
    const Eigen::Array<T, dim, 1> max_e = x1.array().max(x2.array());
    const Eigen::Array<T, dim, 1> min_e = x1.array().min(x2.array());
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
MUDA_GENERIC bool point_triangle_cd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                               const Eigen::Matrix<T, 3, 1>& t0,
                                               const Eigen::Matrix<T, 3, 1>& t1,
                                               const Eigen::Matrix<T, 3, 1>& t2,
                                               T dist)
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
MUDA_GENERIC bool edge_edge_cd_broadphase(const Eigen::Matrix<T, 3, 1>& ea0,
                                          const Eigen::Matrix<T, 3, 1>& ea1,
                                          const Eigen::Matrix<T, 3, 1>& eb0,
                                          const Eigen::Matrix<T, 3, 1>& eb1,
                                          T                             dist)
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
MUDA_GENERIC bool point_triangle_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                                const Eigen::Matrix<T, 3, 1>& t0,
                                                const Eigen::Matrix<T, 3, 1>& t1,
                                                const Eigen::Matrix<T, 3, 1>& t2,
                                                const Eigen::Matrix<T, 3, 1>& dp,
                                                const Eigen::Matrix<T, 3, 1>& dt0,
                                                const Eigen::Matrix<T, 3, 1>& dt1,
                                                const Eigen::Matrix<T, 3, 1>& dt2,
                                                T dist)
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
MUDA_GENERIC bool edge_edge_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& ea0,
                                           const Eigen::Matrix<T, 3, 1>& ea1,
                                           const Eigen::Matrix<T, 3, 1>& eb0,
                                           const Eigen::Matrix<T, 3, 1>& eb1,
                                           const Eigen::Matrix<T, 3, 1>& dea0,
                                           const Eigen::Matrix<T, 3, 1>& dea1,
                                           const Eigen::Matrix<T, 3, 1>& deb0,
                                           const Eigen::Matrix<T, 3, 1>& deb1,
                                           T                             dist)
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
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                            const Eigen::Matrix<T, 3, 1>& e0,
                                            const Eigen::Matrix<T, 3, 1>& e1,
                                            const Eigen::Matrix<T, 3, 1>& dp,
                                            const Eigen::Matrix<T, 3, 1>& de0,
                                            const Eigen::Matrix<T, 3, 1>& de1,
                                            T                             dist)
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
MUDA_GENERIC bool point_point_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p0,
                                             const Eigen::Matrix<T, 3, 1>& p1,
                                             const Eigen::Matrix<T, 3, 1>& dp0,
                                             const Eigen::Matrix<T, 3, 1>& dp1,
                                             T                             dist)
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
MUDA_GENERIC bool point_triangle_ccd(Eigen::Matrix<T, 3, 1> p,
                                     Eigen::Matrix<T, 3, 1> t0,
                                     Eigen::Matrix<T, 3, 1> t1,
                                     Eigen::Matrix<T, 3, 1> t2,
                                     Eigen::Matrix<T, 3, 1> dp,
                                     Eigen::Matrix<T, 3, 1> dt0,
                                     Eigen::Matrix<T, 3, 1> dt1,
                                     Eigen::Matrix<T, 3, 1> dt2,
                                     T                      eta,
                                     T                      thickness,
                                     int                    max_iter,
                                     T&                     toc)
{
    Eigen::Matrix<T, 3, 1> mov = (dt0 + dt1 + dt2 + dp) / 4;
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

    T dist2_cur;
    point_triangle_distance_unclassified(p, t0, t1, t2, dist2_cur);
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
        point_triangle_distance_unclassified(p, t0, t1, t2, dist2_cur);
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
MUDA_GENERIC bool edge_edge_ccd(Eigen::Matrix<T, 3, 1> ea0,
                                Eigen::Matrix<T, 3, 1> ea1,
                                Eigen::Matrix<T, 3, 1> eb0,
                                Eigen::Matrix<T, 3, 1> eb1,
                                Eigen::Matrix<T, 3, 1> dea0,
                                Eigen::Matrix<T, 3, 1> dea1,
                                Eigen::Matrix<T, 3, 1> deb0,
                                Eigen::Matrix<T, 3, 1> deb1,
                                T                      eta,
                                T                      thickness,
                                int                    max_iter,
                                T&                     toc)
{
    Eigen::Matrix<T, 3, 1> mov = (dea0 + dea1 + deb0 + deb1) / 4;
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

    T dist2_cur;
    edge_edge_distance_unclassified(ea0, ea1, eb0, eb1, dist2_cur);
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
        edge_edge_distance_unclassified(ea0, ea1, eb0, eb1, dist2_cur);
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
MUDA_GENERIC bool point_edge_ccd(const Eigen::Matrix<T, 2, 1>& x0,
                                 const Eigen::Matrix<T, 2, 1>& x1,
                                 const Eigen::Matrix<T, 2, 1>& x2,
                                 const Eigen::Matrix<T, 2, 1>& d0,
                                 const Eigen::Matrix<T, 2, 1>& d1,
                                 const Eigen::Matrix<T, 2, 1>& d2,
                                 T                             eta,
                                 T&                            toc)
{
    T a = d0[0] * (d2[1] - d1[1]) + d0[1] * (d1[0] - d2[0]) + d2[0] * d1[1]
          - d2[1] * d1[0];
    T b = x0[0] * (d2[1] - d1[1]) + d0[0] * (x2[1] - x1[1])
          + d0[1] * (x1[0] - x2[0]) + x0[1] * (d1[0] - d2[0]) + d1[1] * x2[0]
          + d2[0] * x1[1] - d1[0] * x2[1] - d2[1] * x1[0];
    T c = x0[0] * (x2[1] - x1[1]) + x0[1] * (x1[0] - x2[0]) + x2[0] * x1[1]
          - x2[1] * x1[0];

    T   roots[2];
    int rootAmt = 0;
    if(a == 0)
    {
        if(b == 0)
        {
            // parallel motion, only need to handle colinear case
            if(c == 0)
            {
                // colinear PP CCD
                if((x0 - x1).dot(d0 - d1) < 0)
                {
                    roots[rootAmt] =
                        sqrt((x0 - x1).squaredNorm() / (d0 - d1).squaredNorm());
                    if(roots[rootAmt] > 0 && roots[rootAmt] <= 1)
                    {
                        ++rootAmt;
                    }
                }
                if((x0 - x2).dot(d0 - d2) < 0)
                {
                    roots[rootAmt] =
                        sqrt((x0 - x2).squaredNorm() / (d0 - d2).squaredNorm());
                    if(roots[rootAmt] > 0 && roots[rootAmt] <= 1)
                    {
                        ++rootAmt;
                    }
                }

                if(rootAmt == 2)
                {
                    toc = std::min(roots[0], roots[1]) * (1 - eta);
                    return true;
                }
                else if(rootAmt == 1)
                {
                    toc = roots[0] * (1 - eta);
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            rootAmt  = 1;
            roots[0] = -c / b;
        }
    }
    else
    {
        T delta = b * b - 4 * a * c;
        if(delta == 0)
        {
            rootAmt  = 1;
            roots[0] = -b / (2 * a);
        }
        else if(delta > 0)
        {
            rootAmt = 2;
            // accurate expression differs in b's sign
            if(b > 0)
            {
                roots[0] = (-b - sqrt(delta)) / (2 * a);
                roots[1] = 2 * c / (-b - sqrt(delta));
            }
            else
            {
                roots[0] = 2 * c / (-b + sqrt(delta));
                roots[1] = (-b + sqrt(delta)) / (2 * a);
            }

            if(roots[0] > roots[1])
            {
                thrust::swap(roots[0], roots[1]);
            }
        }
    }

    for(int i = 0; i < rootAmt; ++i)
    {
        if(roots[i] > 0 && roots[i] <= 1)
        {
            // check overlap
            T ratio;
            if(point_edge_distance_type(Eigen::Matrix<T, 2, 1>(x0 + roots[i] * d0),
                                        Eigen::Matrix<T, 2, 1>(x1 + roots[i] * d1),
                                        Eigen::Matrix<T, 2, 1>(x2 + roots[i] * d2),
                                        ratio)
               == PointEdgeDistanceType::PE)
            {
                toc = roots[i] * (1 - eta);  //TODO: distance eta
                return true;
            }
        }
    }

    return false;
}

template <typename T>
MUDA_GENERIC bool point_edge_ccd(Eigen::Matrix<T, 3, 1> p,
                                 Eigen::Matrix<T, 3, 1> e0,
                                 Eigen::Matrix<T, 3, 1> e1,
                                 Eigen::Matrix<T, 3, 1> dp,
                                 Eigen::Matrix<T, 3, 1> de0,
                                 Eigen::Matrix<T, 3, 1> de1,
                                 T                      eta,
                                 T                      thickness,
                                 int                    max_iter,
                                 T&                     toc)
{
    Eigen::Matrix<T, 3, 1> mov = (dp + de0 + de1) / 3;
    de0 -= mov;
    de1 -= mov;
    dp -= mov;
    T maxDispMag = dp.norm() + sqrt(std::max(de0.squaredNorm(), de1.squaredNorm()));
    if(maxDispMag == 0)
    {
        return false;
    }

    T dist2_cur;
    point_edge_distance_unclassified(p, e0, e1, dist2_cur);
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
        point_edge_distance_unclassified(p, e0, e1, dist2_cur);
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
MUDA_GENERIC bool point_point_ccd(Eigen::Matrix<T, 3, 1> p0,
                                  Eigen::Matrix<T, 3, 1> p1,
                                  Eigen::Matrix<T, 3, 1> dp0,
                                  Eigen::Matrix<T, 3, 1> dp1,
                                  T                      eta,
                                  T                      thickness,
                                  int                    max_iter,
                                  T&                     toc)
{
    Eigen::Matrix<T, 3, 1> mov = (dp0 + dp1) / 2;
    dp1 -= mov;
    dp0 -= mov;
    T maxDispMag = dp0.norm() + dp1.norm();
    if(maxDispMag == 0)
    {
        return false;
    }

    T dist2_cur;
    point_point_distance(p0, p1, dist2_cur);
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
        point_point_distance(p0, p1, dist2_cur);
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
}  // namespace muda::distance