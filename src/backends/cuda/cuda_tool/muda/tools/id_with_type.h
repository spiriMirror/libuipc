#pragma once
#include <cstdint>
#include <ostream>
#include <muda/muda_def.h>
#undef max
namespace muda
{
template <typename T = uint64_t>
class IdWithType
{
  public:
    using value_type                 = T;
    static constexpr auto invalid_id = std::numeric_limits<value_type>::max();
    MUDA_GENERIC explicit IdWithType(value_type value) noexcept
        : m_value{value}
    {
    }
    MUDA_GENERIC explicit IdWithType() noexcept
        : m_value{invalid_id}
    {
    }
    MUDA_GENERIC value_type value() const noexcept { return m_value; }
    friend std::ostream&    operator<<(std::ostream& os, const IdWithType& id)
    {
        os << id.m_value;
        return os;
    }
    MUDA_GENERIC friend bool operator==(const IdWithType& lhs, const IdWithType& rhs) noexcept
    {
        return lhs.m_value == rhs.m_value;
    }
    MUDA_GENERIC friend bool operator!=(const IdWithType& lhs, const IdWithType& rhs) noexcept
    {
        return lhs.m_value != rhs.m_value;
    }
    MUDA_GENERIC friend bool operator<(const IdWithType& lhs, const IdWithType& rhs) noexcept
    {
        return lhs.m_value < rhs.m_value;
    }
    MUDA_GENERIC friend bool operator>(const IdWithType& lhs, const IdWithType& rhs) noexcept
    {
        return lhs.m_value > rhs.m_value;
    }
    MUDA_GENERIC bool is_valid() const noexcept
    {
        return m_value != invalid_id;
    }

  protected:
    value_type m_value{invalid_id};
};

using U64IdWithType = IdWithType<uint64_t>;
using U32IdWithType = IdWithType<uint32_t>;
using I64IdWithType = IdWithType<int64_t>;
using I32IdWithType = IdWithType<int32_t>;
}  // namespace muda

namespace std
{
template <typename T>
struct hash<muda::IdWithType<T>>
{
    size_t operator()(const muda::IdWithType<T>& s) const noexcept
    {
        return std::hash<T>{}(s.value());
    }
};
}  // namespace std