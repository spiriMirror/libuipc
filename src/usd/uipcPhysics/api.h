//
// Copyright 2017 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#ifndef UIPCPHYSICS_API_H
#define UIPCPHYSICS_API_H

#include "pxr/base/arch/export.h"

#if defined(PXR_STATIC)
#   define UIPCPHYSICS_API
#   define UIPCPHYSICS_API_TEMPLATE_CLASS(...)
#   define UIPCPHYSICS_API_TEMPLATE_STRUCT(...)
#   define UIPCPHYSICS_LOCAL
#else
#   if defined(UIPCPHYSICS_EXPORTS)
#       define UIPCPHYSICS_API ARCH_EXPORT
#       define UIPCPHYSICS_API_TEMPLATE_CLASS(...) ARCH_EXPORT_TEMPLATE(class, __VA_ARGS__)
#       define UIPCPHYSICS_API_TEMPLATE_STRUCT(...) ARCH_EXPORT_TEMPLATE(struct, __VA_ARGS__)
#   else
#       define UIPCPHYSICS_API ARCH_IMPORT
#       define UIPCPHYSICS_API_TEMPLATE_CLASS(...) ARCH_IMPORT_TEMPLATE(class, __VA_ARGS__)
#       define UIPCPHYSICS_API_TEMPLATE_STRUCT(...) ARCH_IMPORT_TEMPLATE(struct, __VA_ARGS__)
#   endif
#   define UIPCPHYSICS_LOCAL ARCH_HIDDEN
#endif

#endif
