#include <uipc/geometry/attribute_collection_commit.h>
#include <uipc/geometry/attribute_collection_factory.h>

namespace uipc::geometry
{
AttributeCollectionCommit::AttributeCollectionCommit(const AttributeCollection& current,
                                                     const AttributeCollection& reference)
{
    for(auto&& [name, attr_slot] : current.m_attributes)
    {
        // check if the attribute is evolving
        // if so, we need to copy it to the diff_copy
        if(attr_slot->is_evolving())
        {
            m_attribute_collection.share(name, *attr_slot, attr_slot->allow_destroy());
        }

        // check if the attribute is newer than the reference
        // if so we also need to copy it to the diff_copy
        auto ref_attribute = reference.find(name);
        if(ref_attribute)
        {
            if(attr_slot->last_modified() > ref_attribute->last_modified())
            {
                m_attribute_collection.share(name, *attr_slot, attr_slot->allow_destroy());
            }
        }
        else
        {
            // if the attribute is not in the reference, we need to copy it to the diff_copy
            m_attribute_collection.share(name, *attr_slot, attr_slot->allow_destroy());
        }
    }

    auto cn = current.names();
    std::ranges::sort(cn);
    auto rn = reference.names();
    std::ranges::sort(rn);
    m_removed_names.reserve(rn.size());

    // compute: ref - cur
    // e.g.
    // ref = {"A", "B", "C"}
    // cur = {"A", "D"}
    // removed_names = {"B", "C"}
    std::ranges::set_difference(rn, cn, std::back_inserter(m_removed_names));
}

AttributeCollectionCommit::AttributeCollectionCommit(const AttributeCollection& dst)
    : m_attribute_collection{dst}
{
}

AttributeCollectionCommit operator-(const AttributeCollection& dst,
                                    const AttributeCollection& src)
{
    return AttributeCollectionCommit{dst, src};
}

AttributeCollection& operator+=(AttributeCollection& dst, const AttributeCollectionCommit& inc)
{
    dst.update_from(inc);
    return dst;
}
}  // namespace uipc::geometry