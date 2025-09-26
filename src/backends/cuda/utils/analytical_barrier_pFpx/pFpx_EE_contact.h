#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::analyticalBarrier
{


template <class T>
MUDA_GENERIC void analytical_edge_edge_pFpx(const Eigen::Vector<T, 3>& e0,
                                            const Eigen::Vector<T, 3>& e1,
                                            const Eigen::Vector<T, 3>& e2,
                                            const Eigen::Vector<T, 3>& e3,
                                            T d_hatSqrt,
                                            T result[12][9]);


}

#include "details/pFpx_EE_contact.inl"