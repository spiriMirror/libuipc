#pragma once
#include <muda/ext/linear_system/common.h>
#include <muda/view/view_base.h>
#include <muda/ext/linear_system/bsr_matrix_view.h>

namespace muda
{
template <bool IsConst, typename Ty>
using CSRMatrixViewT = BSRMatrixViewT<IsConst, Ty, 1>;

template <typename Ty>
using CSRMatrixView = CSRMatrixViewT<false, Ty>;
template <typename Ty>
using CCSRMatrixView = CSRMatrixViewT<true, Ty>;
}  // namespace muda

#include "details/csr_matrix_view.inl"
