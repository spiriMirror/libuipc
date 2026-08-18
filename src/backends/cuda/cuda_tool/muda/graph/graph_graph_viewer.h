#pragma once
#include <muda/graph/graph_viewer.h>
#include <muda/compute_graph/compute_graph_var.h>

namespace muda
{
template <>
class ComputeGraphVar<GraphViewer> : public ComputeGraphVarBase
{
  public:
    using ROView = GraphViewer;
    using RWView = GraphViewer;

  protected:
    friend class ComputeGraph;
    friend class ComputeGraphVarManager;

    using ComputeGraphVarBase::ComputeGraphVarBase;

    ComputeGraphVar(ComputeGraphVarManager* var_manager, std::string_view name, VarId var_id) MUDA_NOEXCEPT
        : ComputeGraphVarBase(var_manager, name, var_id)
    {
    }

    ComputeGraphVar(ComputeGraphVarManager* var_manager,
                    std::string_view        name,
                    VarId                   var_id,
                    const RWView&           init_value) MUDA_NOEXCEPT
        : ComputeGraphVarBase(var_manager, name, var_id, true),
          m_value(init_value)
    {
    }

    virtual ~ComputeGraphVar() = default;

  public:
    ROView ceval() const { return _ceval(m_value); }
    RWView eval() { return _eval(m_value); }

    operator ROView() const { return ceval(); }
    operator RWView() { return eval(); }

    void                          update(const RWView& view);
    ComputeGraphVar<GraphViewer>& operator=(const RWView& view);

    virtual void graphviz_def(std::ostream& o,
                              const ComputeGraphGraphvizOptions& options) const override;

  private:
    RWView m_value;
};

}  // namespace muda

#include "details/graph_graph_viewer.inl"