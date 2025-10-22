#pragma once
#include "coloring_algorithm.hpp"
#include <vector>
#include <fmt/ranges.h>

using GraphColoring::GraphColor;

namespace GraphColoring
{
class Dsatur final : public GraphColor
{
  public:
    using GraphColor::GraphColor;

  private:
    void do_solve() override;
};
}  // namespace GraphColoring
