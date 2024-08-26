namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void point_point_distance_unclassified(const Eigen::Vector<T, 3>& p0,
                                                    const Eigen::Vector<T, 3>& p1,
                                                    T& dist2)
{
    return point_point_distance(p0, p1, dist2);
}

template <class T>
MUDA_GENERIC void point_triangle_distance_unclassified(const Eigen::Vector<T, 3>& p,
                                                       const Eigen::Vector<T, 3>& t0,
                                                       const Eigen::Vector<T, 3>& t1,
                                                       const Eigen::Vector<T, 3>& t2,
                                                       T& dist2)
{
    switch(point_triangle_distance_type(p, t0, t1, t2))
    {
        case PointTriangleDistanceType::PP_PT0: {
            point_point_distance(p, t0, dist2);
            break;
        }

        case PointTriangleDistanceType::PP_PT1: {
            point_point_distance(p, t1, dist2);
            break;
        }

        case PointTriangleDistanceType::PP_PT2: {
            point_point_distance(p, t2, dist2);
            break;
        }

        case PointTriangleDistanceType::PE_PT0T1: {
            point_edge_distance(p, t0, t1, dist2);
            break;
        }

        case PointTriangleDistanceType::PE_PT1T2: {
            point_edge_distance(p, t1, t2, dist2);
            break;
        }

        case PointTriangleDistanceType::PE_PT2T0: {
            point_edge_distance(p, t2, t0, dist2);
            break;
        }

        case PointTriangleDistanceType::PT: {
            point_triangle_distance(p, t0, t1, t2, dist2);
            break;
        }

        default:
            MUDA_KERNEL_ERROR_WITH_LOCATION("invalid type");
            break;
    }
}

template <class T>
MUDA_GENERIC void edge_edge_distance_unclassified(const Eigen::Vector<T, 3>& ea0,
                                                  const Eigen::Vector<T, 3>& ea1,
                                                  const Eigen::Vector<T, 3>& eb0,
                                                  const Eigen::Vector<T, 3>& eb1,
                                                  T& dist2)
{
    switch(edge_edge_distance_type(ea0, ea1, eb0, eb1))
    {
        case EdgeEdgeDistanceType::PP_Ea0Eb0: {
            point_point_distance(ea0, eb0, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PP_Ea0Eb1: {
            point_point_distance(ea0, eb1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PE_Ea0Eb0Eb1: {
            point_edge_distance(ea0, eb0, eb1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PP_Ea1Eb0: {
            point_point_distance(ea1, eb0, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PP_Ea1Eb1: {
            point_point_distance(ea1, eb1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PE_Ea1Eb0Eb1: {
            point_edge_distance(ea1, eb0, eb1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PE_Eb0Ea0Ea1: {
            point_edge_distance(eb0, ea0, ea1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::PE_Eb1Ea0Ea1: {
            point_edge_distance(eb1, ea0, ea1, dist2);
            break;
        }

        case EdgeEdgeDistanceType::EE: {
            edge_edge_distance(ea0, ea1, eb0, eb1, dist2);
            break;
        }

        default:
            MUDA_KERNEL_ERROR_WITH_LOCATION("invalid type");
            break;
    }
}

// http://geomalgorithms.com/a02-_lines.html
template <class T>
MUDA_GENERIC void point_edge_distance_unclassified(const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& e0,
                                                   const Eigen::Vector<T, 3>& e1,
                                                   T& dist2)
{
    switch(point_edge_distance_type(p, e0, e1))
    {
        case PointEdgeDistanceType::PP_PE0: {
            point_point_distance(p, e0, dist2);
            break;
        }

        case PointEdgeDistanceType::PP_PE1: {
            point_point_distance(p, e1, dist2);
            break;
        }

        case PointEdgeDistanceType::PE: {
            point_edge_distance(p, e0, e1, dist2);
            break;
        }
        default:
            MUDA_KERNEL_ERROR_WITH_LOCATION("invalid type");
            break;
    }
}

}  // namespace muda::distance