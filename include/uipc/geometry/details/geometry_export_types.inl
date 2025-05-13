// NOTE: Don't add #pragma once here, this file is meant to be included multiple times.
#ifndef UIPC_GEOMETRY_EXPORT_DEF

// You need to define UIPC_ATTRIBUTE_EXPORT_DEF before including this file.
#error "UIPC_GEOMETRY_EXPORT_DEF is not defined"

#else

UIPC_GEOMETRY_EXPORT_DEF(Geometry);
UIPC_GEOMETRY_EXPORT_DEF(SimplicialComplex);
UIPC_GEOMETRY_EXPORT_DEF(ImplicitGeometry);

#endif