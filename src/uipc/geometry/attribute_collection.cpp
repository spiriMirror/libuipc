#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>
#include <uipc/common/set.h>
#include <uipc/common/list.h>
#include <uipc/common/range.h>
#include <iostream>
namespace uipc::geometry
{
S<IAttributeSlot> AttributeCollection::share(std::string_view name, const IAttributeSlot& slot)
{
    auto n  = string{name};
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
    auto it = m_attributes.find(string{name});
    if(it == m_attributes.end())
    {
        UIPC_WARN_WITH_LOCATION("Destroying non-existing attribute [{}]", name);
        return;
    }

    if(!it->second->allow_destroy())
        throw GeometryAttributeError{fmt::format("Attribute [{}] don't allow destroy!", name)};

    m_attributes.erase(it);
}

S<IAttributeSlot> AttributeCollection::find(std::string_view name)
{
    auto it = m_attributes.find(string{name});
    return it != m_attributes.end() ? it->second : nullptr;
}


S<const IAttributeSlot> AttributeCollection::find(std::string_view name) const
{
    auto it = m_attributes.find(string{name});
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
                                    const AttributeCopy&       copy,
                                    span<const string>    _include_names,
                                    span<const string>    _exclude_names)
{
    vector<string> include_names;
    vector<string> exclude_names(_exclude_names.begin(), _exclude_names.end());
    vector<string> filtered_names;

    if(_include_names.empty())
        include_names = other.names();

    filtered_names.reserve(include_names.size());

    std::ranges::sort(include_names);
    if(!exclude_names.empty())
    {
        std::ranges::sort(exclude_names);
        std::ranges::set_difference(include_names, exclude_names, std::back_inserter(filtered_names));
    }
    else
    {
        filtered_names = std::move(include_names);
    }


    for(auto& name : filtered_names)
    {
        auto it = other.m_attributes.find(name);
        if(it == other.m_attributes.end())
            throw GeometryAttributeError{
                fmt::format("Attribute [{}] not found in the source sim_systems.", name)};

        auto other_slot = it->second;

        // optimize for the case that the attribute size is the same
        if(copy.type() == AttributeCopy::CopyType::SameDim)
        {
            // just share
            UIPC_ASSERT(this->size() == other.size(),
                        "Attribute size mismatch, "
                        "dst size is {}, src size is {}, Did you forget to resize the dst attribute sim_systems?",
                        this->size(),
                        other.size());

            m_attributes[name] = other_slot->clone();

            continue;
        }

        // if the name is not found in the current collection, create a new slot
        if(auto this_it = m_attributes.find(name); this_it == m_attributes.end())
        {
            auto c             = other_slot->do_clone_empty();
            m_attributes[name] = c;
            UIPC_ASSERT(c->is_shared() == false, "The attribute is shared, why can it happen?");
            c->attribute().resize(size());
            c->attribute().copy_from(other_slot->attribute(), copy);
        }
        else  // the name is found in the current collection
        {

            this_it->second->make_owned();
            this_it->second->attribute().copy_from(other_slot->attribute(), copy);
        }
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

vector<string> AttributeCollection::names() const
{
    vector<string> names;
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

Json AttributeCollection::to_json() const
{
    Json j = Json::array();
    for(auto& [name, slot] : m_attributes)
    {
        j.push_back(slot->to_json());
    }
    return j;
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