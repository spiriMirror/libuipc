//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "./constitutionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<UipcPhysicsConstitutionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
UipcPhysicsConstitutionAPI::~UipcPhysicsConstitutionAPI()
{
}

/* static */
UipcPhysicsConstitutionAPI
UipcPhysicsConstitutionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UipcPhysicsConstitutionAPI();
    }
    return UipcPhysicsConstitutionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind UipcPhysicsConstitutionAPI::_GetSchemaKind() const
{
    return UipcPhysicsConstitutionAPI::schemaKind;
}

/* static */
bool
UipcPhysicsConstitutionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<UipcPhysicsConstitutionAPI>(whyNot);
}

/* static */
UipcPhysicsConstitutionAPI
UipcPhysicsConstitutionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<UipcPhysicsConstitutionAPI>()) {
        return UipcPhysicsConstitutionAPI(prim);
    }
    return UipcPhysicsConstitutionAPI();
}

/* static */
const TfType &
UipcPhysicsConstitutionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<UipcPhysicsConstitutionAPI>();
    return tfType;
}

/* static */
bool 
UipcPhysicsConstitutionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
UipcPhysicsConstitutionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
UipcPhysicsConstitutionAPI::GetConstitution_uidAttr() const
{
    return GetPrim().GetAttribute(UipcPhysicsTokens->uipcConstitution_uid);
}

UsdAttribute
UipcPhysicsConstitutionAPI::CreateConstitution_uidAttr(VtValue const &defaultValue, bool writeSparsely) const
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
UipcPhysicsConstitutionAPI::GetSchemaAttributeNames(bool includeInherited)
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
