#pragma once
#include <cinttypes>
#include <string_view>
namespace muda
{
enum class ComputeGraphNodeType : uint8_t
{
    None,
    KernelNode,
    MemcpyNode,
    MemsetNode,
    CaptureNode,
    EventRecordNode,
    EventWaitNode,
    Max
};

inline std::string_view enum_name(ComputeGraphNodeType t)
{
    switch(t)
    {
        case ComputeGraphNodeType::None:
            return "None";
        case ComputeGraphNodeType::KernelNode:
            return "KernelNode";
        case ComputeGraphNodeType::MemcpyNode:
            return "MemcpyNode";
        case ComputeGraphNodeType::MemsetNode:
            return "MemsetNode";
        case ComputeGraphNodeType::CaptureNode:
            return "CaptureNode";
        case ComputeGraphNodeType::EventRecordNode:
            return "EventRecordNode";
        case ComputeGraphNodeType::EventWaitNode:
            return "EventWaitNode";
        default:
            return "Unknown";
    }
}
}  // namespace muda