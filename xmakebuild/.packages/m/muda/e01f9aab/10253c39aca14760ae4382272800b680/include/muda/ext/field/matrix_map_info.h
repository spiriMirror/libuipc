#pragma once

namespace muda
{
/// <summary>
/// For MapMatrix e.g. Eigen::Map< ... >
/// </summary>
template <typename T, int M, int N>
class MatrixMapInfo
{
  public:
    T*  begin;
    int outer_stride;
    int inner_stride;
};

template <typename T, int M, int N>
class CMatrixMapInfo
{
  public:
    const T*  begin;
    int outer_stride;
    int inner_stride;
};
}  // namespace muda