//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "./scene.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<UipcPhysicsScene,
        TfType::Bases< UsdTyped > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysicsScene")
    // to find TfType<UipcPhysicsScene>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, UipcPhysicsScene>("PhysicsScene");
}

/* virtual */
UipcPhysicsScene::~UipcPhysicsScene()
{
}

/* static */
UipcPhysicsScene
UipcPhysicsScene::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UipcPhysicsScene();
    }
    return UipcPhysicsScene(stage->GetPrimAtPath(path));
}

/* static */
UipcPhysicsScene
UipcPhysicsScene::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysicsScene");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UipcPhysicsScene();
    }
    return UipcPhysicsScene(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind UipcPhysicsScene::_GetSchemaKind() const
{
    return UipcPhysicsScene::schemaKind;
}

/* static */
const TfType &
UipcPhysicsScene::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<UipcPhysicsScene>();
    return tfType;
}

/* static */
bool 
UipcPhysicsScene::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
UipcPhysicsScene::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
UipcPhysicsScene::GetGravityAttr() const
{
    return GetPrim().GetAttribute(UipcPhysicsTokens->physicsGravity);
}

UsdAttribute
UipcPhysicsScene::CreateGravityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(UipcPhysicsTokens->physicsGravity,
                       SdfValueTypeNames->Vector3f,
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
UipcPhysicsScene::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        UipcPhysicsTokens->physicsGravity,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdTyped::GetSchemaAttributeNames(true),
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
