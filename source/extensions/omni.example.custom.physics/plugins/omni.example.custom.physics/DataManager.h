// SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#ifdef _MSC_VER
#pragma warning(disable : 4005) // to disable NOMINMAX warning
#endif
#include "PhysicsUtils.h"
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/SimStageWithHistory.h>

using namespace omni::fabric;
using TransformationCache = std::unordered_map<uint64_t, std::pair<carb::Float3, carb::Float4>>;

namespace omni {
namespace example {
namespace custom {
namespace physics {
class DataManager {
    TransformationCache mInitialTransformation;
    omni::fabric::TokenC mWorldPosToken;
    omni::fabric::TokenC mWorldOrientToken;
    omni::fabric::TokenC mWorldScaleToken;
    omni::fabric::TokenC mMujocoBodyToken;
    omni::fabric::Type mFloat1Type;
    omni::fabric::Type mFloat3Type;
    omni::fabric::Type mDouble3Type;
    omni::fabric::Type mQuatType;
    omni::fabric::Type mFloat3ArrayType;
    std::string gWorldPositionTokenString = "_worldPosition";
    std::string gWorldOrientationTokenString = "_worldOrientation";
    std::string gWorldScaleTokenString = "_worldScale";
    std::string gMujocoBodyString = "MujocoBody";

  public:
    omni::fabric::UsdStageId mStageId;
    DataManager();
    ~DataManager();
    void updateUsd(MujocoData &mMujoco, UsdData &mUsdData, pxr::UsdStageRefPtr mStage, bool mStageInitialized,
                   float currentTime, float elapsedSecs);
    void updateFabric(MujocoData &mMujoco, UsdData &mUsdData, pxr::UsdStageRefPtr mStage, bool mStageInitialized,
                      float currentTime, float elapsedSecs);
    void initializeFabricRigidBody(pxr::UsdGeomXformCache &xfCache, const pxr::UsdPrim &prim,
                                   IStageReaderWriter *iStageReaderWriter, StageReaderWriterId stageInProgress);

  private:
    double mRemainderTime = 0;

};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni