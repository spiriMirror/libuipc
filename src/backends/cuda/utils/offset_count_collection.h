#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>

namespace uipc::backend::cuda
{
template <std::integral T>
class OffsetCountCollection
{
  public:
    void resize(SizeT N);

    SizeT         total_count() const;
    span<const T> counts() const;
    span<T>       counts();

    span<const T> offsets() const;
    span<T>       offsets();

    void scan();

  private:
    vector<T> m_counts;
    vector<T> m_offsets;
    SizeT     N = 0;
};
}  // namespace uipc::backend::cuda
