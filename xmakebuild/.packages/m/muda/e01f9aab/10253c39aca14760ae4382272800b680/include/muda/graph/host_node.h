#pragma once
#include <muda/graph/graph_base.h>

namespace muda
{
class HostNode : public GraphNode
{
  public:
    using this_type = HostNode;
    friend class Graph;
};

template <typename T>
class HostNodeParms : public NodeParms
{
    cudaHostNodeParams m_parms;

  public:
    T hostData;
    using this_type = HostNodeParms;
    friend class Graph;
    friend class std::shared_ptr<this_type>;
    friend class std::unique_ptr<this_type>;
    friend class std::weak_ptr<this_type>;

    HostNodeParms(const T& f)
        : hostData(f)
        , m_parms({})
    {
        m_parms.userData = &hostData;
    }
    auto fn() const { return m_parms.fn; }
    void fn(cudaHostFn_t fn) { m_parms.fn = fn; }
    auto userdata() const { return m_parms.userData; }
    void userdata(void* userdata) { m_parms.userData = userdata; }
    const cudaHostNodeParams* handle() const { return &m_parms; }
    cudaHostNodeParams*       handle() { return &m_parms; }
};
}  // namespace muda