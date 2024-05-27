#pragma once
#include <uipc/world/contact_element.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/exception.h>
namespace uipc::world
{
void apply(const ContactElement& c, geometry::SimplicialComplex& sc);
}
