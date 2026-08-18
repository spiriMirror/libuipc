#pragma once
#include <cub/util_type.cuh>
namespace muda
{
template <typename Key, typename Value>
using KeyValuePair = cub::KeyValuePair<Key, Value>;
}