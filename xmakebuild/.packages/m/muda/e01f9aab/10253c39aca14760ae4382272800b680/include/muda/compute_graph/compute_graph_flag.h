#pragma once
namespace muda
{
enum class ComputeGraphFlag
{
    HostLaunch   = 1,
    DeviceLaunch = 2 | HostLaunch,
};
}