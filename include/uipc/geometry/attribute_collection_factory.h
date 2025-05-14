#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeCollectionFactory
{
    class Impl;

  public:
    AttributeCollectionFactory();
    ~AttributeCollectionFactory();

    /**
     * @breif Collect all attributes that are `evolving` from the source attribute collection.
     */
    [[nodiscard]] S<AttributeCollection> collect_evolving(const AttributeCollection& source);

    /**
     * @brief Collect all names of the attributes that are deleting from the reference attribute collection.
     * 
     * retNames = referenceNames - currentNames
     * 
     * @param current The current attribute collection.
     * @param reference The reference attribute collection.
     * @return A vector of strings representing the names of the attributes that are deleting.
     */
    [[nodiscard]] vector<std::string> collect_removing(const AttributeCollection& current,
                                                       const AttributeCollection& reference);

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

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
