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
    void           H(const diff_sim::SparseCOOView& value);
    void           pGpP(const diff_sim::SparseCOOView& value);
    void           need_backend_broadcast(bool v);
    bool           need_backend_broadcast() const;
    void           need_backend_clear(bool v);
    bool           need_backend_clear() const;
    core::DiffSim& ref();

  private:
    core::DiffSim& m_diff_sim;
};
}  // namespace uipc::backend