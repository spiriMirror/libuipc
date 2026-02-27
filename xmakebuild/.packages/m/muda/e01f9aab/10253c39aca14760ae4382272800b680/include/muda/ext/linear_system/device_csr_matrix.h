#pragma once
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/device_bsr_matrix.h>
#include <cusparse.h>
#include <muda/ext/linear_system/csr_matrix_view.h>

namespace muda::details
{
template <typename T, int M, int N>
class MatrixFormatConverter;
}

namespace muda
{
template <typename T>
using DeviceCSRMatrix = DeviceBSRMatrix<T, 1>;
}  // namespace muda
#include "details/device_csr_matrix.inl"
