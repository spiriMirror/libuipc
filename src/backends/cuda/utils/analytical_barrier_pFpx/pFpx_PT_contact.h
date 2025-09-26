#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::analyticalBarrier
{


template <class T>
MUDA_GENERIC void analytical_point_triangle_pFpx(const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& t1,
                                                   const Eigen::Vector<T, 3>& t2,
                                                   const Eigen::Vector<T, 3>& t3,
                                                   T d_hatSqrt,
                                                   T result[12][9]);


}

#include "details/pFpx_PT_contact.inl"