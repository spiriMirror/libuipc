#pragma once
#include <uipc/core/subscene_element.h>
#include <uipc/core/subscene_model.h>
#include <uipc/common/json.h>
#include <uipc/common/span.h>
#include <uipc/core/subscene_model_collection.h>

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::backend
{
class SubsceneTabularVisitor;
}

namespace uipc::core
{
class UIPC_CORE_API SubsceneTabular final
{
    friend class uipc::backend::SubsceneTabularVisitor;
    friend class SceneSnapshot;
    friend class Scene;
    friend class internal::Scene;

  public:
    SubsceneTabular() noexcept;
    ~SubsceneTabular() noexcept;
    // delete copy_from
    SubsceneTabular(const SubsceneTabular&)            = delete;
    SubsceneTabular& operator=(const SubsceneTabular&) = delete;

    SubsceneElement create(std::string_view name = "") noexcept;

    IndexT insert(const SubsceneElement& L,
                  const SubsceneElement& R,
                  bool                   enable = true,
                  const Json&            config = default_config());

    SubsceneModel at(SizeT i, SizeT j) const;

    void default_model(bool enable, const Json& config = default_config()) noexcept;

    SubsceneElement default_element() noexcept;
    SubsceneModel   default_model() const noexcept;

    SubsceneModelCollection  subscene_models() noexcept;
    CSubsceneModelCollection subscene_models() const noexcept;

    friend UIPC_CORE_API void to_json(Json& j, const SubsceneTabular& ct);

    SizeT element_count() const noexcept;

    static Json default_config() noexcept;

  private:
    class Impl;
    U<Impl> m_impl;
    friend class SceneFactory;
    geometry::AttributeCollection& internal_subscene_models() const noexcept;
    span<SubsceneElement>          subscene_elements() const noexcept;
    void build_from(const geometry::AttributeCollection& ac, span<const SubsceneElement> ce);
    void update_from(const geometry::AttributeCollectionCommit& acc,
                     span<const SubsceneElement>                ce);
};

UIPC_CORE_API void to_json(Json& j, const SubsceneTabular& ct);
}  // namespace uipc::core
