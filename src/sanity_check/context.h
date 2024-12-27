#include <sanity_checker.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::sanity_check
{
using uipc::core::SanityCheckResult;

class ContactTabular
{
  public:
    ContactTabular() noexcept = default;
    void                      init(backend::SceneVisitor& s);
    const core::ContactModel& at(IndexT i, IndexT j) const;
    SizeT                     element_count() const noexcept;

    // delete copy constructor
    ContactTabular(const ContactTabular&) = delete;
    // delete copy assignment
    ContactTabular& operator=(const ContactTabular&) = delete;

  private:
    vector<core::ContactModel> m_table;
    SizeT                      m_contact_element_count = 0;
};

class Context final : public SanityChecker
{
  public:
    explicit Context(SanityCheckerCollection& c, core::Scene& s) noexcept;
    virtual ~Context() override;

    const geometry::SimplicialComplex& scene_simplicial_surface() const noexcept;
    const ContactTabular& contact_tabular() const noexcept;

  private:
    friend class SanityCheckerCollection;
    void prepare();
    void destroy();

    virtual U64 get_id() const noexcept override;

    virtual SanityCheckResult do_check(backend::SceneVisitor&,
                                       backend::SanityCheckMessageVisitor&) override;

    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::sanity_check
