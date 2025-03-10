#include <uipc/diff_sim/parameter_collection.h>
#include <uipc/core/object.h>
#include <uipc/common/list.h>
#include <uipc/common/zip.h>
#include <uipc/common/span_as_eigen.h>

namespace uipc::diff_sim
{
class IConnection
{
  public:
    virtual S<geometry::IAttributeSlot> diff_parm_slot() const             = 0;
    virtual S<geometry::IAttributeSlot> parm_slot() const                  = 0;
    virtual void                        broadcast(span<const Float> parms) = 0;
};

template <int M, int N>
class Connection : public IConnection
{
  public:
    using ValueI = std::conditional_t<M == 1 && N == 1, IndexT, Matrix<IndexT, M, N>>;
    using ValueF = std::conditional_t<M == 1 && N == 1, Float, Matrix<Float, M, N>>;

    Connection(S<geometry::AttributeSlot<ValueI>> diff_parm_slot,
               S<geometry::AttributeSlot<ValueF>> parm_slot)
    {
        m_diff_parm_slot = diff_parm_slot;
        m_parm_slot      = parm_slot;
    }

    virtual S<geometry::IAttributeSlot> diff_parm_slot() const override
    {
        return m_diff_parm_slot;
    }

    virtual S<geometry::IAttributeSlot> parm_slot() const override
    {
        return m_parm_slot;
    }

    virtual void broadcast(span<const Float> parms) override
    {
        auto diff_parm_view = m_diff_parm_slot->view();
        auto parm_view      = view(*m_parm_slot);

        for(auto&& [diff_parm, parm] : zip(diff_parm_view, parm_view))
        {
            if constexpr(M == 1 && N == 1)
            {
                if(diff_parm >= 0 && diff_parm < parms.size())
                    parm = parms[diff_parm];
            }
            else
            {
                for(SizeT i = 0; i < M; ++i)
                {
                    for(SizeT j = 0; j < N; ++j)
                    {
                        auto& eI = diff_parm(i, j);
                        auto& eF = parm(i, j);
                        if(eI >= 0 && eI < parms.size())
                            eF = parms[eI];
                    }
                }
            }
        }
    }

  private:
    S<geometry::AttributeSlot<ValueI>> m_diff_parm_slot;
    S<geometry::AttributeSlot<ValueF>> m_parm_slot;
};

template <int M, int N>
S<IConnection> make_connection(S<geometry::IAttributeSlot> diff_parm_slot,
                               S<geometry::IAttributeSlot> parm_slot)
{
    using ValueI = std::conditional_t<M == 1 && N == 1, IndexT, Matrix<IndexT, M, N>>;
    using ValueF = std::conditional_t<M == 1 && N == 1, Float, Matrix<Float, M, N>>;

    auto diff_parm_slot_i =
        std::dynamic_pointer_cast<geometry::AttributeSlot<ValueI>>(diff_parm_slot);
    auto parm_slot_f =
        std::dynamic_pointer_cast<geometry::AttributeSlot<ValueF>>(parm_slot);

    if(diff_parm_slot_i && parm_slot_f)
        return uipc::make_shared<Connection<M, N>>(diff_parm_slot_i, parm_slot_f);
    else
        return nullptr;
}

class ParameterCollection::Impl
{
  public:
    span<const Float>      view() const { return parms; }
    span<Float>            view() { return parms; }
    vector<Float>          parms;
    list<S<IConnection>>   connection_buffer;
    vector<S<IConnection>> connections;
    bool                   built                  = false;
    bool                   need_backend_broadcast = false;

    void resize(SizeT N, Float default_value)
    {
        parms.resize(N, default_value);
    }

    void broadcast()
    {
        if(!built)
        {
            UIPC_WARN_WITH_LOCATION("ParameterCollection is not built yet, ignore broadcast.");
        }
        else
        {
            for(auto&& connection : connections)
                connection->broadcast(parms);

            need_backend_broadcast = true;
        }
    }

    template <int M, int N>
    bool try_push_back(S<geometry::IAttributeSlot> diff_parm_slot,
                       S<geometry::IAttributeSlot> parm_slot)
    {
        auto connection = make_connection<M, N>(diff_parm_slot, parm_slot);
        if(connection)
            connection_buffer.push_back(connection);
        return connection != nullptr;
    }

    void connect(S<geometry::IAttributeSlot> diff_parm_slot, S<geometry::IAttributeSlot> parm_slot)
    {
#define TRY_PUSH_BACK(M, N)                                                    \
    if(try_push_back<M, N>(diff_parm_slot, parm_slot))                         \
        return;

        // scalar
        TRY_PUSH_BACK(1, 1);

        // vector
        TRY_PUSH_BACK(2, 1);
        TRY_PUSH_BACK(3, 1);
        TRY_PUSH_BACK(4, 1);
        TRY_PUSH_BACK(6, 1);
        TRY_PUSH_BACK(9, 1);
        TRY_PUSH_BACK(12, 1);

        // matrix
        TRY_PUSH_BACK(2, 2);
        TRY_PUSH_BACK(3, 3);
        TRY_PUSH_BACK(4, 4);
        TRY_PUSH_BACK(6, 6);
        TRY_PUSH_BACK(9, 9);
        TRY_PUSH_BACK(12, 12);

#undef TRY_PUSH_BACK

        UIPC_ASSERT(false,
                    "Unsupported Diff Parameter Attribute [{}<{}>], [{}<{}>] ",
                    diff_parm_slot->name(),
                    diff_parm_slot->type_name(),
                    parm_slot->name(),
                    parm_slot->type_name());
    }

    void build()
    {
        connections.resize(connection_buffer.size());
        std::ranges::move(connection_buffer, connections.begin());
        connection_buffer.clear();
        built = true;
    }
};


ParameterCollection::ParameterCollection()
    : m_impl{uipc::make_unique<Impl>()}
{
}

ParameterCollection::~ParameterCollection() {}

void ParameterCollection::resize(SizeT N, Float default_value)
{
    m_impl->resize(N, default_value);
}

void ParameterCollection::broadcast()
{
    m_impl->broadcast();
}

span<const Float> ParameterCollection::view() const
{
    return m_impl->view();
}

Eigen::Map<const Eigen::Matrix<Float, Eigen::Dynamic, 1>> ParameterCollection::as_eigen() const
{
    return uipc::as_eigen(view());
}

SizeT ParameterCollection::size() const
{
    return m_impl->parms.size();
}

void ParameterCollection::connect(S<geometry::IAttributeSlot> diff_parm_slot,
                                  S<geometry::IAttributeSlot> parm_slot)
{
    m_impl->connect(diff_parm_slot, parm_slot);
}

void ParameterCollection::build()
{
    m_impl->build();
}

bool ParameterCollection::need_backend_broadcast() const
{
    return m_impl->need_backend_broadcast;
}

void ParameterCollection::need_backend_broadcast(bool v)
{
    m_impl->need_backend_broadcast = v;
}

span<Float> view(ParameterCollection& collection)
{
    return collection.m_impl->view();
}

Eigen::Map<Eigen::Matrix<Float, Eigen::Dynamic, 1>> as_eigen(ParameterCollection& collection)
{
    return uipc::as_eigen(view(collection));
}
}  // namespace uipc::diff_sim
