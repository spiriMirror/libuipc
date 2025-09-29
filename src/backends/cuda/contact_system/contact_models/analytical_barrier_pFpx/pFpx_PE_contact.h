#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::analyticalBarrier
{


template <class T>
MUDA_GENERIC void analytical_point_edge_pFpx(const Eigen::Vector<T, 3>& p,
                                             const Eigen::Vector<T, 3>& e1,
                                             const Eigen::Vector<T, 3>& e2,
                                             T d_hatSqrt,
                                             T result[9][4]);


}

#include "details/pFpx_PE_contact.inl"