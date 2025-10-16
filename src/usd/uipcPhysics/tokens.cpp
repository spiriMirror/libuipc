//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "./tokens.h"

PXR_NAMESPACE_OPEN_SCOPE

UipcPhysicsTokensType::UipcPhysicsTokensType() :
    uipcConstitution_uid("uipc:constitution_uid", TfToken::Immortal),
    AffineBodyAPI("AffineBodyAPI", TfToken::Immortal),
    ConstitutionAPI("ConstitutionAPI", TfToken::Immortal),
    allTokens({
        uipcConstitution_uid,
        AffineBodyAPI,
        ConstitutionAPI
    })
{
}

TfStaticData<UipcPhysicsTokensType> UipcPhysicsTokens;

PXR_NAMESPACE_CLOSE_SCOPE
