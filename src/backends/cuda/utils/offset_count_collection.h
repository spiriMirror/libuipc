#pragma once
#include <type_define.h>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>

namespace uipc::backend::cuda
{
/*
 * @breif A Offsets/Counts Collection
 * 
 * You can only setup the `Counts`, and get the `Offsets` after calling `scan()`.
 * The `scan()` function will compute the offsets based on the counts.
 * 
 */
template <std::integral T>
class OffsetCountCollection
{
  public:
    void    resize(SizeT N);
    span<T> counts();
    void    scan(const T& init = T{0});

    SizeT            total_count() const;
    span<const T>    counts() const;
    span<const T>    offsets() const;
    std::tuple<T, T> operator[](SizeT i) const;

  private:
    vector<T> m_counts;
    vector<T> m_offsets;
    SizeT     m_N = 0;
};
}  // namespace uipc::backend::cuda
