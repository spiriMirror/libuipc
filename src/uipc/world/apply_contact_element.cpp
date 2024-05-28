#include <uipc/world/apply_contact_element.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::world
{
void apply(const ContactElement& c, geometry::SimplicialComplex& sc)
{
    auto ces = sc.meta().find<IndexT>(builtin::contact_element_id);
    if(!ces)
    {
        ces = sc.meta().create<IndexT>(builtin::contact_element_id, c.id());
    }
    else
    {
        auto view    = geometry::view(*ces);
        view.front() = c.id();
    }
}
}  // namespace uipc::world