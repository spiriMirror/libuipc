#include <app/app.h>
#include <backends/common/details/dependency_cycle.h>
#include <array>
#include <vector>

namespace
{
struct Node
{
    int                id      = 0;
    bool               enabled = true;
    std::vector<Node*> dependencies;
};

std::vector<Node*> find_cycle(const std::array<Node*, 4>& nodes)
{
    return uipc::backend::detail::find_dependency_cycle<Node*>(
        nodes,
        [](Node* node) -> const std::vector<Node*>&
        { return node->dependencies; },
        [](Node* node) { return node->enabled; });
}
}  // namespace

TEST_CASE("dependency cycle detection", "[algorithm][dependency]")
{
    Node a{.id = 0};
    Node b{.id = 1};
    Node c{.id = 2};
    Node d{.id = 3};

    const std::array nodes{&a, &b, &c, &d};

    a.dependencies = {&b, &d};
    b.dependencies = {&c};
    REQUIRE(find_cycle(nodes).empty());

    c.dependencies = {&b};
    REQUIRE(find_cycle(nodes) == std::vector<Node*>{&b, &c, &b});

    b.enabled = false;
    REQUIRE(find_cycle(nodes).empty());

    d.dependencies = {&d};
    REQUIRE(find_cycle(nodes) == std::vector<Node*>{&d, &d});
}
