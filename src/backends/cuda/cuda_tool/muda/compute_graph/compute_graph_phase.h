#pragma once
namespace muda
{
enum class ComputeGraphPhase
{
    None,
    TopoBuilding, // we don't create cuda graph at this point, just build the topo
    Building, // we create cuda graph at this point
    Updating, // we update the graph at this point
    SerialLaunching, // we just launch invoke all the graph closure in serial
    Max
};
}