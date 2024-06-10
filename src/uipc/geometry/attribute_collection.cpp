#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>
#include <uipc/common/set.h>
#include <uipc/common/list.h>

namespace uipc::geometry
{
P<IAttributeSlot> AttributeCollection::share(std::string_view name, const IAttributeSlot& slot)
{
    auto n  = std::string{name};
    auto it = m_attributes.find(n);

    if(size() != slot.size())
        throw GeometryAttributeError{
            fmt::format("Attribute size mismatch, "
                        "Attribute Collision size is {}, input slot size is {}.",
                        size(),
                        slot.size())};

    if(it != m_attributes.end())
        throw GeometryAttributeError{
            fmt::format("Attribute with name [{}] already exist!", name)};
    return m_attributes[n] = slot.clone();
}

void AttributeCollection::destroy(std::string_view name)
{
    auto it = m_attributes.find(std::string{name});
    if(it == m_attributes.end())
    {
        UIPC_WARN_WITH_LOCATION("Destroying non-existing attribute [{}]", name);
        return;
    }

    if(!it->second->allow_destroy())
        throw GeometryAttributeError{fmt::format("Attribute [{}] don't allow destroy!", name)};

    m_attributes.erase(it);
}

P<IAttributeSlot> AttributeCollection::find(std::string_view name)
{
    auto it = m_attributes.find(std::string{name});
    return it != m_attributes.end() ? it->second : nullptr;
}


P<const IAttributeSlot> AttributeCollection::find(std::string_view name) const
{
    auto it = m_attributes.find(std::string{name});
    return it != m_attributes.end() ? it->second : nullptr;
}

void AttributeCollection::resize(SizeT N)
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->make_owned();
        slot->attribute().resize(N);
    }
    m_size = N;
}

void AttributeCollection::reorder(span<const SizeT> O)
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->make_owned();
        slot->attribute().reorder(O);
    }
}

void AttributeCollection::copy_from(const AttributeCollection& other,
                                    span<const SizeT>          O,
                                    span<const std::string>    include_names,
                                    span<const std::string>    exclude_names)
{
    UIPC_ASSERT(O.size() == size(),
                "Size mismatch, attribute size ({}), your New2Old mapping ({}). Resize the topology before copy.",
                O.size(),
                size());

    // quick path
    // default copy all attributes
    // if some attribtues exist in this collection, just skip it
    if(include_names.empty() && exclude_names.empty()) [[likely]]
    {
        for(auto& [name, slot] : other.m_attributes)
        {
            // if the name is not found in the current collection, create a new slot
            if(m_attributes.find(name) == m_attributes.end())
            {
                auto c             = slot->do_clone_empty();
                m_attributes[name] = c;
                UIPC_ASSERT(c->is_shared() == false,
                            "The attribute is shared, why can it happen?");
                c->attribute().resize(size());
                c->attribute().copy_from(slot->attribute(), O);
            }
        }

        return;
    }

    // at least one of the include_names or exclude_names is not empty

    vector<std::string> filtered_include_names;
    do
    {
        auto names =
            include_names.empty() ?
                other.names() :
                vector<std::string>{include_names.begin(), include_names.end()};

        if(exclude_names.empty()) [[likely]]
        {
            filtered_include_names = std::move(names);
            break;
        }

        vector<std::string> sorted_exclude_names{exclude_names.begin(),
                                                 exclude_names.end()};

        filtered_include_names.reserve(names.size());
        std::ranges::sort(names);
        std::ranges::sort(sorted_exclude_names);
        std::ranges::set_difference(names,
                                    sorted_exclude_names,
                                    std::back_inserter(filtered_include_names));
    } while(0);

    for(const auto& name : filtered_include_names)
    {
        auto it = other.m_attributes.find(name);
        UIPC_ASSERT(it != other.m_attributes.end(),
                    "Attribute [{}] not found in the source collection.",
                    name);

        auto& slot         = it->second;
        auto  c            = slot->do_clone_empty();
        m_attributes[name] = c;
        UIPC_ASSERT(c->is_shared() == false, "The attribute is shared, why can it happen?");
        c->attribute().resize(size());
        c->attribute().copy_from(slot->attribute(), O);
    }
}

SizeT AttributeCollection::size() const
{
    return m_size;
}

void AttributeCollection::clear()
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->make_owned();
        slot->attribute().clear();
    }
}

void AttributeCollection::reserve(SizeT N)
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->attribute().reserve(N);
    }
}

vector<std::string> AttributeCollection::names() const
{
    vector<std::string> names;
    names.reserve(m_attributes.size());
    for(auto& [name, slot] : m_attributes)
    {
        names.push_back(name);
    }
    return names;
}

SizeT AttributeCollection::attribute_count() const
{
    return m_attributes.size();
}

AttributeCollection::AttributeCollection(const AttributeCollection& o)
{
    for(auto& [name, attr] : o.m_attributes)
    {
        m_attributes[name] = attr->clone();
    }
    m_size = o.m_size;
}

AttributeCollection& AttributeCollection::operator=(const AttributeCollection& o)
{
    if(std::addressof(o) == this)
        return *this;
    for(auto& [name, attr] : o.m_attributes)
    {
        m_attributes[name] = attr->clone();
    }
    m_size = o.m_size;
    return *this;
}

AttributeCollection::AttributeCollection(AttributeCollection&& o) noexcept
    : m_attributes(std::move(o.m_attributes))
    , m_size(o.m_size)
{
    o.m_size = 0;
}

AttributeCollection& AttributeCollection::operator=(AttributeCollection&& o) noexcept
{
    if(std::addressof(o) == this)
        return *this;
    m_attributes = std::move(o.m_attributes);
    m_size       = o.m_size;
    o.m_size     = 0;
    return *this;
}

}  // namespace uipc::geometry

namespace fmt
{
appender formatter<uipc::geometry::AttributeCollection>::format(
    const uipc::geometry::AttributeCollection& collection, format_context& ctx)
{
    auto size = collection.size();

    fmt::format_to(ctx.out(), "[");

    for(const auto& [name, slot] : collection.m_attributes)
    {
        std::string_view star = slot->allow_destroy() ? "" : "*";
        fmt::format_to(ctx.out(), "{}'{}':<{}> ", star, name, slot->type_name());
    }

    fmt::format_to(ctx.out(), "]");

    return ctx.out();
}
}  // namespace fmt