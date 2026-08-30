#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace uipc::backend::detail
{
template <typename Node, typename NodeRange, typename Dependencies, typename Enabled>
std::vector<Node> find_dependency_cycle(const NodeRange& nodes,
                                        Dependencies&&   dependencies,
                                        Enabled&&        enabled)
{
    enum class VisitState : std::uint8_t
    {
        Unvisited,
        Visiting,
        Visited
    };

    std::unordered_map<Node, VisitState> states;
    std::vector<Node>                    stack;
    std::vector<Node>                    cycle;

    std::function<bool(Node)> visit = [&](Node node)
    {
        if(!enabled(node))
            return false;

        states[node] = VisitState::Visiting;
        stack.push_back(node);

        for(Node dependency : dependencies(node))
        {
            if(!enabled(dependency))
                continue;

            const auto state = states.contains(dependency) ? states.at(dependency) :
                                                             VisitState::Unvisited;
            if(state == VisitState::Unvisited)
            {
                if(visit(dependency))
                    return true;
            }
            else if(state == VisitState::Visiting)
            {
                const auto cycle_begin = std::ranges::find(stack, dependency);
                cycle.assign(cycle_begin, stack.end());
                cycle.push_back(dependency);
                return true;
            }
        }

        stack.pop_back();
        states[node] = VisitState::Visited;
        return false;
    };

    for(Node node : nodes)
    {
        if(!enabled(node))
            continue;

        const auto state = states.contains(node) ? states.at(node) : VisitState::Unvisited;
        if(state == VisitState::Unvisited && visit(node))
            break;
    }

    return cycle;
}
}  // namespace uipc::backend::detail
