#pragma once
#include <muda/ext/linear_system/doublet_vector_viewer.h>
namespace muda
{
template <typename T, int N>
using BCOOVectorViewer = CDoubletVectorViewer<T, N>;

template <typename T, int N>
using CBCOOVectorViewer = CDoubletVectorViewer<T, N>;

template <typename T>
using COOVectorViewer = BCOOVectorViewer<T, 1>;

template <typename T>
using CCOOVectorViewer = CBCOOVectorViewer<T, 1>;
}  // namespace muda

#include "details/bcoo_vector_viewer.inl"
