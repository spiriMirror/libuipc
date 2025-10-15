//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#ifndef UIPCPHYSICS_TOKENS_H
#define UIPCPHYSICS_TOKENS_H

/// \file uipcPhysics/tokens.h

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// 
// This is an automatically generated file (by usdGenSchema.py).
// Do not hand-edit!
// 
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#include "pxr/pxr.h"
#include "./api.h"
#include "pxr/base/tf/staticData.h"
#include "pxr/base/tf/token.h"
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE


/// \class UipcPhysicsTokensType
///
/// \link UipcPhysicsTokens \endlink provides static, efficient
/// \link TfToken TfTokens\endlink for use in all public USD API.
///
/// These tokens are auto-generated from the module's schema, representing
/// property names, for when you need to fetch an attribute or relationship
/// directly by name, e.g. UsdPrim::GetAttribute(), in the most efficient
/// manner, and allow the compiler to verify that you spelled the name
/// correctly.
///
/// UipcPhysicsTokens also contains all of the \em allowedTokens values
/// declared for schema builtin attributes of 'token' scene description type.
/// Use UipcPhysicsTokens like so:
///
/// \code
///     gprim.GetMyTokenValuedAttr().Set(UipcPhysicsTokens->physicsGravity);
/// \endcode
struct UipcPhysicsTokensType {
    UIPCPHYSICS_API UipcPhysicsTokensType();
    /// \brief "physics:gravity"
    /// 
    /// UipcPhysicsScene
    const TfToken physicsGravity;
    /// \brief "PhysicsScene"
    /// 
    /// Schema identifer and family for UipcPhysicsScene
    const TfToken PhysicsScene;
    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

/// \var UipcPhysicsTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa UipcPhysicsTokensType
extern UIPCPHYSICS_API TfStaticData<UipcPhysicsTokensType> UipcPhysicsTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
