#include <uipc/common/json_eigen.h>
#include <uipc/common/type_traits.h>

namespace uipc::geometry
{
template <typename T>
Attribute<T>::Attribute(const T& default_value) noexcept
    : m_values{}
    , m_default_value{default_value}
{
}

template <typename T>
span<const T> Attribute<T>::view() const noexcept
{
    return m_values;
}

template <typename T>
std::string Attribute<T>::type() noexcept
{
    return readable_type_name<T>();
}

template <typename T>
SizeT Attribute<T>::get_size() const
{
    return m_values.size();
}

template <typename T>
backend::BufferView Attribute<T>::get_backend_view() const noexcept
{
    return m_backend_view;
}

template <typename T>
std::string_view Attribute<T>::get_type_name() const noexcept
{
    thread_local static std::string type_name = readable_type_name<T>();
    return type_name;
}

template <typename T>
void Attribute<T>::do_resize(SizeT N)
{
    m_values.resize(N, this->m_default_value);
}

template <typename T>
void Attribute<T>::do_clear()
{
    m_values.clear();
}

template <typename T>
void Attribute<T>::do_reserve(SizeT N)
{
    m_values.reserve(N);
}

template <typename T>
S<IAttribute> Attribute<T>::do_clone() const
{
    return uipc::make_shared<Attribute<T>>(*this);
}
template <typename T>
inline S<IAttribute> Attribute<T>::do_clone_empty() const
{
    return uipc::make_shared<Attribute<T>>(m_default_value);
}
template <typename T>
void Attribute<T>::do_reorder(span<const SizeT> O) noexcept
{
    auto old_values = m_values;
    for(SizeT i = 0; i < O.size(); ++i)
        m_values[i] = old_values[O[i]];
}

template <typename T>
void Attribute<T>::do_copy_from(const IAttribute& other, const AttributeCopy& copy) noexcept
{
    auto& other_attr = static_cast<const Attribute<T>&>(other);
    copy.template copy<T>(m_values, other_attr.m_values);
}

template <typename T>
Json Attribute<T>::do_to_json(SizeT i) const noexcept
{
    Json j;
    if constexpr(requires(T t) { Json{t}; })
    {
        j = m_values[i];
    }
    else if constexpr(requires(T t) { t.to_json(); })
    {
        j = m_values[i].to_json();
    }
    else if constexpr(requires(T t) { fmt::format("{}", t); })
    {
        j = fmt::format("{}", m_values[i]);
    }
    else
    {
        j = fmt::format("<{}>", typeid(T).name());
    }
    return j;
}

template <typename T>
void Attribute<T>::do_from_json(const Json& j) const noexcept
{
    UIPC_ASSERT(j.is_array(), "To create an Attribute, this json must be an array");
    if constexpr(requires(T t) {
                     Json j;
                     j.get<T>();  // can get<T>()
                 })
    {
        m_values.resize(j.size());
        for(SizeT i = 0; i < j.size(); ++i)
        {
            m_values[i] = j[i].get<T>();
        }
    }
    else if constexpr(requires(T t) { t.from_json(j); })
    {
        m_values.resize(j.size());
        for(SizeT i = 0; i < j.size(); ++i)
        {
            m_values[i].from_json(j[i]);
        }
    }
    else
    {
        static_assert("Attribute<T>::from_json: T must be a type that can be converted from json");
    }
}

//template <typename T>
//const BufferInfo& Attribute<T>::buffer_info() const noexcept
//{
//    m_buffer_info.data     = (void*)m_values.data();
//    m_buffer_info.shape[0] = m_values.size();
//    return m_buffer_info;
//}
}  // namespace uipc::geometry
