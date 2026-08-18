#pragma once
namespace muda
{
enum class ComputeGraphVarUsage : char
{
    None,
    Read,
    ReadWrite,
    Max
};

inline bool operator<(ComputeGraphVarUsage lhs, ComputeGraphVarUsage rhs)
{
    return static_cast<char>(lhs) < static_cast<char>(rhs);
}

inline bool operator<=(ComputeGraphVarUsage lhs, ComputeGraphVarUsage rhs)
{
    return static_cast<char>(lhs) <= static_cast<char>(rhs);
}

inline bool operator>(ComputeGraphVarUsage lhs, ComputeGraphVarUsage rhs)
{
    return static_cast<char>(lhs) > static_cast<char>(rhs);
}

inline bool operator>=(ComputeGraphVarUsage lhs, ComputeGraphVarUsage rhs)
{
    return static_cast<char>(lhs) >= static_cast<char>(rhs);
}

inline bool operator==(ComputeGraphVarUsage lhs, ComputeGraphVarUsage rhs)
{
    return static_cast<char>(lhs) == static_cast<char>(rhs);
}
}  // namespace muda