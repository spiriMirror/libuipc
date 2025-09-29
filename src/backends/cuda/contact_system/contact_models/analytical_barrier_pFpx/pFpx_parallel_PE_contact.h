#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::analyticalBarrier
{


template <class T>
MUDA_GENERIC void analytical_parallel_point_edge_pFpx(const Eigen::Vector<T, 3>& e0,
                                                     const Eigen::Vector<T, 3>& e2,
                                                     const Eigen::Vector<T, 3>& e3,
                                                     const Eigen::Vector<T, 3>& e1,
                                                     T d_hatSqrt,
                                                     T result[12][9]);


}

#include "details/pFpx_parallel_PE_contact.inl"