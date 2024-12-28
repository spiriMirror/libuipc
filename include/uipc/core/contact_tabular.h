#pragma once
#include <uipc/core/contact_element.h>
#include <uipc/core/contact_model.h>
#include <uipc/common/json.h>
#include <uipc/common/span.h>

namespace uipc::core
{
class UIPC_CORE_API ContactTabular
{
  public:
    ContactTabular() noexcept;
    ~ContactTabular() noexcept;
    // delete copy_from
    ContactTabular(const ContactTabular&)            = delete;
    ContactTabular& operator=(const ContactTabular&) = delete;

    ContactElement& create(std::string_view name = "") noexcept;

    void insert(const ContactElement& L,
                const ContactElement& R,
                Float                 friction_rate,
                Float                 resistance,
                bool                  enable = true,
                const Json&           config = default_config());

    ContactModel at(SizeT i, SizeT j) const;

    void default_model(Float       friction_rate,
                       Float       resistance,
                       const Json& config = default_config()) noexcept;

    ContactElement& default_element() noexcept;

    const ContactModel& default_model() const noexcept;

    std::span<const ContactModel> contact_models() const noexcept;

    friend void to_json(Json& j, const ContactTabular& ct);

    SizeT element_count() const noexcept;

    static Json default_config() noexcept;

  private:
    class Impl;
    U<Impl> m_impl;
};

void to_json(Json& j, const ContactTabular& ct);
}  // namespace uipc::core
