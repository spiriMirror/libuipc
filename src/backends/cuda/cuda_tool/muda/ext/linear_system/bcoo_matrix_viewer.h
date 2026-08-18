#pragma once
#include <muda/ext/linear_system/triplet_matrix_viewer.h>
namespace muda
{
template <typename T, int M, int N>
using BCOOMatrixViewer = CTripletMatrixViewer<T, M, N>;

template <typename T, int M, int N>
using CBCOOMatrixViewer = CTripletMatrixViewer<T, M, N>;

template <typename T>
using COOMatrixViewer = BCOOMatrixViewer<T, 1, 1>;

template <typename T>
using CCOOMatrixViewer = CBCOOMatrixViewer<T, 1, 1>;
}  // namespace muda

#include "details/bcoo_matrix_viewer.inl"
