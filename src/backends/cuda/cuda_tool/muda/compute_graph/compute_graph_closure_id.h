#pragma once
#include <muda/tools/id_with_type.h>
namespace muda
{
class ClosureId : public U64IdWithType
{
    using U64IdWithType::U64IdWithType;
};
}  // namespace muda

namespace std
{
template <>
struct hash<muda::ClosureId>
{
    size_t operator()(const muda::ClosureId& s) const noexcept
    {
        return std::hash<uint64_t>{}(s.value());
    }
};
}  // namespace std