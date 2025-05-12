namespace uipc::geometry
{
template <std::derived_from<Geometry> T>
T* Geometry::as()
{
    return dynamic_cast<T*>(this);
}

template <std::derived_from<Geometry> T>
const T* Geometry::as() const
{
    return dynamic_cast<const T*>(this);
}

template <bool IsConst>
Json uipc::geometry::Geometry::MetaAttributesT<IsConst>::to_json() const
{
    return m_attributes.to_json();
}

template <bool IsConst>
    void Geometry::InstanceAttributesT<IsConst>::resize(size_t size)
    && requires(!IsConst) { m_attributes.resize(size); }

    template <bool IsConst>
    void Geometry::InstanceAttributesT<IsConst>::reserve(size_t size)
    && requires(!IsConst) { m_attributes.reserve(size); }

    template <bool IsConst>
    void Geometry::InstanceAttributesT<IsConst>::clear()
    && requires(!IsConst) { m_attributes.clear(); }

    template <bool IsConst>
    SizeT Geometry::InstanceAttributesT<IsConst>::size() &&
{
    return m_attributes.size();
}

template <bool IsConst>
    void Geometry::InstanceAttributesT<IsConst>::destroy(std::string_view name)
    && requires(!IsConst) { m_attributes.destroy(name); } template <bool IsConst>
    Json uipc::geometry::Geometry::InstanceAttributesT<IsConst>::to_json() const
{
    return m_attributes.to_json();
}
}  // namespace uipc::geometry
