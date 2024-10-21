#include <uipc/diff_sim/parameter_collection.h>
#include <uipc/core/object.h>

namespace uipc::diff_sim
{
class ParameterCollection::Impl
{
  public:
    void resize(SizeT N, Float default_value)
    {
        m_values.resize(N, default_value);
    }

    span<const Float> view() const { return m_values; }

    span<Float> view() { return m_values; }

    vector<Float> m_values;
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

span<const Float> ParameterCollection::view() const
{
    return m_impl->view();
}

span<Float> view(ParameterCollection& collection)
{
    return collection.m_impl->view();
}
}  // namespace uipc::diff_sim
