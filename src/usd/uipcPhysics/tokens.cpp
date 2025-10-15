//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "./tokens.h"

PXR_NAMESPACE_OPEN_SCOPE

UipcPhysicsTokensType::UipcPhysicsTokensType() :
    physicsGravity("physics:gravity", TfToken::Immortal),
    PhysicsScene("PhysicsScene", TfToken::Immortal),
    allTokens({
        physicsGravity,
        PhysicsScene
    })
{
}

TfStaticData<UipcPhysicsTokensType> UipcPhysicsTokens;

PXR_NAMESPACE_CLOSE_SCOPE
