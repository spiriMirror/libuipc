#include <uipc/backend/visitors/diff_sim_visitor.h>
#include <uipc/core/diff_sim.h>

namespace uipc::backend
{
DiffSimVisitor::DiffSimVisitor(core::DiffSim& diff_sim)
    : m_diff_sim(diff_sim)
{
}

DiffSimVisitor::~DiffSimVisitor() {}

diff_sim::ParameterCollection& DiffSimVisitor::parameters()
{
    return m_diff_sim.parameters();
}

const diff_sim::ParameterCollection& DiffSimVisitor::parameters() const
{
    return m_diff_sim.parameters();
}

core::DiffSim& DiffSimVisitor::ref()
{
    return m_diff_sim;
}
}  // namespace uipc::backend