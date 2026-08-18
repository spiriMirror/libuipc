#pragma once
namespace muda
{
template <typename T>
struct Tag
{
};

struct Default
{
};

using DefaultTag = Tag<Default>;
}  // namespace muda