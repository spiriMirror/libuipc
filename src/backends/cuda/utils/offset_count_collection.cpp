#include <utils/offset_count_collection.h>
#include <uipc/common/log.h>
#include <numeric>

namespace uipc::backend::cuda
{
template <std::integral T>
void OffsetCountCollection<T>::resize(SizeT N)
{
    auto N_1 = N + 1;
    m_counts.resize(N_1, 0);
    m_offsets.resize(N_1, 0);
    this->m_N = N;
}

template <std::integral T>
SizeT OffsetCountCollection<T>::total_count() const
{
    return m_offsets[m_N];
}

template <std::integral T>
span<const T> OffsetCountCollection<T>::counts() const
{
    return span{m_counts}.subspan(0, m_N);
}

template <std::integral T>
span<T> OffsetCountCollection<T>::counts()
{
    return span{m_counts}.subspan(0, m_N);
}

template <std::integral T>
span<const T> OffsetCountCollection<T>::offsets() const
{
    return span{m_offsets}.subspan(0, m_N);
}

template <std::integral T>
void OffsetCountCollection<T>::scan(const T& init)
{
    m_counts[m_N]  = 0;
    m_offsets[m_N] = 0;
    std::exclusive_scan(m_counts.begin(), m_counts.end(), m_offsets.begin(), init);
}

template <std::integral T>
std::tuple<T, T> OffsetCountCollection<T>::operator[](SizeT i) const
{
    UIPC_ASSERT(i < m_N, "Index out of bounds for OffsetCountCollection, i = {}, N = {}", i, m_N);
    return {m_offsets[i], m_counts[i]};
}

template class OffsetCountCollection<I32>;
template class OffsetCountCollection<I64>;
template class OffsetCountCollection<U64>;
template class OffsetCountCollection<U32>;
}  // namespace uipc::backend::cuda
