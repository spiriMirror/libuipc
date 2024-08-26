namespace uipc::backend::cuda::distance
{
template <class T, int dim>
MUDA_GENERIC PointPointDistanceType point_point_distance_type(
    const Eigen::Vector<T, dim>& p0, const Eigen::Vector<T, dim>& p1)
{
    //return 0
    return PointPointDistanceType::PP;
}

template <class T, int dim>
MUDA_GENERIC PointEdgeDistanceType
point_edge_distance_type(const Eigen::Vector<T, dim>& p,
                         const Eigen::Vector<T, dim>& e0,
                         const Eigen::Vector<T, dim>& e1,
                         T&                           ratio)
{
    const Eigen::Vector<T, dim> e = e1 - e0;
    ratio                         = e.dot(p - e0) / e.squaredNorm();
    if(ratio <= 0)
    {
        // return 0;
        return PointEdgeDistanceType::PP_PE0;  // PP (p-e0)
    }
    else if(ratio >= 1)
    {
        // return 1;
        return PointEdgeDistanceType::PP_PE1;  // PP (p-e1)
    }
    else
    {
        // return 2;
        return PointEdgeDistanceType::PE;  // PE
    }
}

template <class T, int dim>
MUDA_GENERIC PointEdgeDistanceType
point_edge_distance_type(const Eigen::Vector<T, dim>& p,
                         const Eigen::Vector<T, dim>& e0,
                         const Eigen::Vector<T, dim>& e1)
{
    T ratio;
    return point_edge_distance_type(p, e0, e1, ratio);
}

template <class T>
MUDA_GENERIC PointTriangleDistanceType
point_triangle_distance_type(const Eigen::Vector<T, 3>& p,
                             const Eigen::Vector<T, 3>& t0,
                             const Eigen::Vector<T, 3>& t1,
                             const Eigen::Vector<T, 3>& t2)
{
    Eigen::Matrix<T, 2, 3> basis;
    basis.row(0) = (t1 - t0).transpose();
    basis.row(1) = (t2 - t0).transpose();

    const Eigen::Vector<T, 3> nVec = basis.row(0).cross(basis.row(1));

    Eigen::Matrix<T, 2, 3> param;

    basis.row(1) = basis.row(0).cross(nVec);
    param.col(0) = (basis * basis.transpose()).inverse() * (basis * (p - t0));
    if(param(0, 0) > 0.0 && param(0, 0) < 1.0 && param(1, 0) >= 0.0)
    {
        // return 3;
        return PointTriangleDistanceType::PE_PT0T1;  // PE t0t1
    }
    else
    {
        basis.row(0) = (t2 - t1).transpose();

        basis.row(1) = basis.row(0).cross(nVec);

        param.col(1) = (basis * basis.transpose()).inverse() * (basis * (p - t1));

        if(param(0, 1) > 0.0 && param(0, 1) < 1.0 && param(1, 1) >= 0.0)
        {
            // return 4;
            return PointTriangleDistanceType::PE_PT1T2;  // PE t1t2
        }
        else
        {
            basis.row(0) = (t0 - t2).transpose();

            basis.row(1) = basis.row(0).cross(nVec);
            param.col(2) = (basis * basis.transpose()).inverse() * (basis * (p - t2));
            if(param(0, 2) > 0.0 && param(0, 2) < 1.0 && param(1, 2) >= 0.0)
            {
                // return 5
                return PointTriangleDistanceType::PE_PT2T0;  // PE t2t0
            }
            else
            {
                if(param(0, 0) <= 0.0 && param(0, 2) >= 1.0)
                {
                    // return 0;
                    return PointTriangleDistanceType::PP_PT0;  // PP t0
                }
                else if(param(0, 1) <= 0.0 && param(0, 0) >= 1.0)
                {
                    // return 1;
                    return PointTriangleDistanceType::PP_PT1;  // PP t1
                }
                else if(param(0, 2) <= 0.0 && param(0, 1) >= 1.0)
                {
                    // return 2;
                    return PointTriangleDistanceType::PP_PT2;  // PP t2
                }
                else
                {
                    // return 6;
                    return PointTriangleDistanceType::PT;  // PT
                }
            }
        }
    }
}

// a more robust implementation of http://geomalgorithms.com/a07-_distance.html
template <class T>
MUDA_GENERIC EdgeEdgeDistanceType edge_edge_distance_type(const Eigen::Vector<T, 3>& ea0,
                                                          const Eigen::Vector<T, 3>& ea1,
                                                          const Eigen::Vector<T, 3>& eb0,
                                                          const Eigen::Vector<T, 3>& eb1)
{
    Eigen::Vector<T, 3> u  = ea1 - ea0;
    Eigen::Vector<T, 3> v  = eb1 - eb0;
    Eigen::Vector<T, 3> w  = ea0 - eb0;
    T                   a  = u.squaredNorm();  // always >= 0
    T                   b  = u.dot(v);
    T                   c  = v.squaredNorm();  // always >= 0
    T                   d  = u.dot(w);
    T                   e  = v.dot(w);
    T                   D  = a * c - b * b;  // always >= 0
    T                   tD = D;  // tc = tN / tD, default tD = D >= 0
    T                   sN, tN;

    EdgeEdgeDistanceType defaultCase = EdgeEdgeDistanceType::EE;  // 8

    // compute the line parameters of the two closest points
    sN = (b * e - c * d);
    if(sN <= 0.0)
    {  // sc < 0 => the s=0 edge is visible
        tN = e;
        tD = c;
        // defaultCase = 2;
        defaultCase = EdgeEdgeDistanceType::PE_Ea0Eb0Eb1;
    }
    else if(sN >= D)
    {  // sc > 1  => the s=1 edge is visible
        tN = e + b;
        tD = c;
        // defaultCase = 5;
        defaultCase = EdgeEdgeDistanceType::PE_Ea1Eb0Eb1;
    }
    else
    {
        tN = (a * e - b * d);
        if(tN > 0.0 && tN < tD
           && (u.cross(v).dot(w) == 0.0 || u.cross(v).squaredNorm() < 1.0e-20 * a * c))
        {
            // if (tN > 0.0 && tN < tD && (u.cross(v).dot(w) == 0.0 || u.cross(v).squaredNorm() == 0.0)) {
            // std::cout << u.cross(v).squaredNorm() / (a * c) << ": " << sN << " " << D << ", " << tN << " " << tD << std::endl;
            // avoid coplanar or nearly parallel EE
            if(sN < D / 2)
            {
                tN = e;
                tD = c;
                // defaultCase = 2;
                defaultCase = EdgeEdgeDistanceType::PE_Ea0Eb0Eb1;
            }
            else
            {
                tN = e + b;
                tD = c;
                // defaultCase = 5;
                defaultCase = EdgeEdgeDistanceType::PE_Ea1Eb0Eb1;
            }
        }
        // else defaultCase stays as 8
    }

    if(tN <= 0.0)
    {  // tc < 0 => the t=0 edge is visible
        // recompute sc for this edge
        if(-d <= 0.0)
        {
            // return 0;
            return EdgeEdgeDistanceType::PP_Ea0Eb0;
        }
        else if(-d >= a)
        {
            // return 3;
            return EdgeEdgeDistanceType::PP_Ea1Eb0;
        }
        else
        {
            // return 6;
            return EdgeEdgeDistanceType::PE_Eb0Ea0Ea1;
        }
    }
    else if(tN >= tD)
    {  // tc > 1  => the t=1 edge is visible
        // recompute sc for this edge
        if((-d + b) <= 0.0)
        {
            // return 1;
            return EdgeEdgeDistanceType::PP_Ea0Eb1;
        }
        else if((-d + b) >= a)
        {
            // return 4;
            return EdgeEdgeDistanceType::PP_Ea1Eb1;
        }
        else
        {
            // return 7;
            return EdgeEdgeDistanceType::PE_Eb1Ea0Ea1;
        }
    }

    return defaultCase;
}

}  // namespace muda::distance