//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "./affineBodyAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<UipcPhysicsAffineBodyAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
UipcPhysicsAffineBodyAPI::~UipcPhysicsAffineBodyAPI()
{
}

/* static */
UipcPhysicsAffineBodyAPI
UipcPhysicsAffineBodyAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UipcPhysicsAffineBodyAPI();
    }
    return UipcPhysicsAffineBodyAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind UipcPhysicsAffineBodyAPI::_GetSchemaKind() const
{
    return UipcPhysicsAffineBodyAPI::schemaKind;
}

/* static */
bool
UipcPhysicsAffineBodyAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<UipcPhysicsAffineBodyAPI>(whyNot);
}

/* static */
UipcPhysicsAffineBodyAPI
UipcPhysicsAffineBodyAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<UipcPhysicsAffineBodyAPI>()) {
        return UipcPhysicsAffineBodyAPI(prim);
    }
    return UipcPhysicsAffineBodyAPI();
}

/* static */
const TfType &
UipcPhysicsAffineBodyAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<UipcPhysicsAffineBodyAPI>();
    return tfType;
}

/* static */
bool 
UipcPhysicsAffineBodyAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
UipcPhysicsAffineBodyAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
UipcPhysicsAffineBodyAPI::GetConstitution_uidAttr() const
{
    return GetPrim().GetAttribute(UipcPhysicsTokens->uipcConstitution_uid);
}

UsdAttribute
UipcPhysicsAffineBodyAPI::CreateConstitution_uidAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(UipcPhysicsTokens->uipcConstitution_uid,
                       SdfValueTypeNames->UInt64,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

namespace {
static inline TfTokenVector
_ConcatenateAttributeNames(const TfTokenVector& left,const TfTokenVector& right)
{
    TfTokenVector result;
    result.reserve(left.size() + right.size());
    result.insert(result.end(), left.begin(), left.end());
    result.insert(result.end(), right.begin(), right.end());
    return result;
}
}

/*static*/
const TfTokenVector&
UipcPhysicsAffineBodyAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        UipcPhysicsTokens->uipcConstitution_uid,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdAPISchemaBase::GetSchemaAttributeNames(true),
            localNames);

    if (includeInherited)
        return allNames;
    else
        return localNames;
}

PXR_NAMESPACE_CLOSE_SCOPE

// ===================================================================== //
// Feel free to add custom code below this line. It will be preserved by
// the code generator.
//
// Just remember to wrap code in the appropriate delimiters:
// 'PXR_NAMESPACE_OPEN_SCOPE', 'PXR_NAMESPACE_CLOSE_SCOPE'.
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--
