#pragma once
#include <uipc/geometry/attribute_collection.h>
#include <uipc/geometry/attribute_collection_commit.h>

namespace uipc::geometry
{


class UIPC_CORE_API AttributeCollectionFactory
{
    class Impl;

  public:
    AttributeCollectionFactory();
    ~AttributeCollectionFactory();

    /**
     * @brief Create an attribute collection from a json object and a pool of shared attributes
     */
    [[nodiscard]] S<AttributeCollection> from_json(const Json& j,
                                                   span<S<IAttributeSlot>> attributes);

    /**
     * @breif Convert an attribute collection to a json object, while using index to replace the attributes pointer
     */
    [[nodiscard]] Json to_json(const AttributeCollection*         acs,
                               unordered_map<IAttribute*, IndexT> attr_to_index,
                               bool evolving_only = false);

    /**
     * @brief Get the difference between the current and reference attribute collections.
     * 
     * - New Attributes and Modified Attributes will be copied to the diff_copy.
     * - Deleted Attributes will be collected in the removed_names.
     * 
     */
    AttributeCollectionCommit diff(const AttributeCollection& current,
                                   const AttributeCollection& reference);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
