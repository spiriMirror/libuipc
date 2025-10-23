#pragma once
#include "coloring_algorithm.hpp"

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
