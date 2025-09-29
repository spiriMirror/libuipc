#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::analyticalBarrier
{


template <class T>
MUDA_GENERIC void analytical_point_point_pFpx(const Eigen::Vector<T, 3>& p0,
                                              const Eigen::Vector<T, 3>& p1,
                                              T d_hatSqrt,
                                              T result[6]);


}

#include "details/pFpx_PP_contact.inl"