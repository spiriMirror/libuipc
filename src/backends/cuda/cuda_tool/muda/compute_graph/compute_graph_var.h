#pragma once
#include <string>
#include <set>
#include <map>
#include <muda/launch/event.h>
#include <muda/mstl/span.h>
#include <muda/type_traits/type_modifier.h>
#include <muda/compute_graph/compute_graph_closure_id.h>
#include <muda/compute_graph/compute_graph_var_usage.h>
#include <muda/compute_graph/compute_graph_var_id.h>
#include <muda/compute_graph/graphviz_options.h>
#include <muda/compute_graph/compute_graph_fwd.h>

namespace muda
{
class ComputeGraphVarBase
{
    std::string_view        m_name;
    ComputeGraphVarManager* m_var_manager = nullptr;
    VarId                   m_var_id;
    bool                    m_is_valid;

  public:
    std::string_view   name() const MUDA_NOEXCEPT { return m_name; }
    VarId              var_id() const MUDA_NOEXCEPT { return m_var_id; }
    bool               is_valid() const MUDA_NOEXCEPT { return m_is_valid; }
    void               update();
    Event::QueryResult query();
    bool               is_using();
    void               sync();
    virtual void       graphviz_def(std::ostream&                      os,
                                    const ComputeGraphGraphvizOptions& options) const;
    virtual void graphviz_id(std::ostream& os, const ComputeGraphGraphvizOptions& options) const;

  protected:
    template <typename RWView>
    RWView _eval(const RWView& view);
    template <typename ROView>
    ROView _ceval(ROView& view) const;

    friend class ComputeGraph;
    friend class ComputeGraphVarManager;

    ComputeGraphVarBase(ComputeGraphVarManager* var_manager,
                        std::string_view        name,
                        VarId var_id) MUDA_NOEXCEPT : m_var_manager(var_manager),
                                                      m_name(name),
                                                      m_var_id(var_id),
                                                      m_is_valid(false)
    {
    }

    ComputeGraphVarBase(ComputeGraphVarManager* var_manager,
                        std::string_view        name,
                        VarId                   var_id,
                        bool is_valid) MUDA_NOEXCEPT : m_var_manager(var_manager),
                                                       m_name(name),
                                                       m_var_id(var_id),
                                                       m_is_valid(is_valid)
    {
    }

    virtual ~ComputeGraphVarBase() = default;


    void base_update();

    friend class LaunchCore;

    mutable std::set<ClosureId> m_closure_ids;

  private:
    void _building_eval(ComputeGraphVarUsage usage) const;
    void base_building_eval();
    void base_building_ceval() const;
    void remove_related_closure_infos(ComputeGraph* graph);

    class RelatedClosureInfo
    {
      public:
        ComputeGraph*       graph;
        std::set<ClosureId> closure_ids;
    };

    mutable std::map<ComputeGraph*, RelatedClosureInfo> m_related_closure_infos;
};

template <typename T>
class ComputeGraphVar : public ComputeGraphVarBase
{
  public:
    static_assert(!std::is_const_v<T>, "T must not be const");
    using ROViewer = read_only_view_t<T>;
    using RWViewer = T;
    static_assert(std::is_convertible_v<RWViewer, ROViewer>,
                  "RWViewer must be convertible to ROView");

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
                    const T&                init_value) MUDA_NOEXCEPT
        : ComputeGraphVarBase(var_manager, name, var_id, true),
          m_value(init_value)
    {
    }

    virtual ~ComputeGraphVar() = default;

  public:
    RWViewer eval() { return _eval(m_value); }
    ROViewer ceval() const { return _ceval(m_value); }

    operator ROViewer() const { return ceval(); }
    operator RWViewer() { return eval(); }

    void                update(const RWViewer& view);
    ComputeGraphVar<T>& operator=(const RWViewer& view);
    virtual void        graphviz_def(std::ostream& os,
                                     const ComputeGraphGraphvizOptions& options) const override;

  private:
    RWViewer m_value;
};

// for host memory
template <typename T>
struct read_only_view<T*>
{
    using type = const T*;
};
template <typename T>
struct read_write_view<const T*>
{
    using type = T*;
};

// for cuda event
template <>
struct read_only_view<cudaEvent_t>
{
    using type = cudaEvent_t;
};
template <>
struct read_write_view<cudaEvent_t>
{
    using type = cudaEvent_t;
};

}  // namespace muda


#include "details/compute_graph_var.inl"