#pragma once
#include <uipc/diff_sim/parameter_collection.h>
#include <uipc/diff_sim/sparse_coo_view.h>

namespace uipc::core
{
class DiffSim;
}
namespace uipc::backend
{
class UIPC_CORE_API DiffSimVisitor
{
  public:
    DiffSimVisitor(core::DiffSim& diff_sim);
    ~DiffSimVisitor();
    diff_sim::ParameterCollection&       parameters();
    const diff_sim::ParameterCollection& parameters() const;
    core::DiffSim&                       ref();

  private:
    core::DiffSim& m_diff_sim;
};
}  // namespace uipc::backend