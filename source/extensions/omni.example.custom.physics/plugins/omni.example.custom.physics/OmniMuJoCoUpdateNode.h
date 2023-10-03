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
#include "DataManager.h"
#include "PhysicsUtils.h"
#include <omni/example/custom/physics/ICustomPhysicsInterface.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

using namespace PXR_NS;
using namespace carb;

namespace omni {
namespace example {
namespace custom {
namespace physics {

class OmniMujocoUpdateNode : public omni::physics::schema::IUsdPhysicsListener,
                             public TfWeakBase,
                             public omni::example::custom::physics::ICustomPhysicsInterface {

    pxr::UsdStageRefPtr mStage = nullptr;
    std::atomic<bool> mStageInitialized = false;
    bool mPaused = true;
    std::string fileName;
    TfNotice::Key mUsdNoticeListenerKey;
    std::vector<UsdGeomXformOp> mXformOps;
    std::vector<UsdGeomXformOp> mScaleOps;
    std::vector<GfVec3d> mScales;
    carb::events::ISubscriptionPtr mUpdateEventsSubscription;
    carb::events::ISubscriptionPtr mTimelineEventsSubscription;

    omni::example::custom::physics::UsdData mUsdData;
    omni::example::custom::physics::MujocoData mMujoco;
    omni::example::custom::physics::DataManager dataManager;

  public:
    // CARB_PLUGIN_INTERFACE("omni.example.custom.physics", 1, 0)
    omni::kit::StageUpdateNode *mStageUpdateNode = nullptr;
    // omni::physics::schema::IUsdPhysicsListener interface
    void parsePrim(const pxr::UsdPrim &prim, omni::physics::schema::ObjectDesc *objectDesc, uint64_t,
                   const pxr::TfTokenVector &appliedApis) override;
    void reportObjectDesc(const pxr::SdfPath &path, const omni::physics::schema::ObjectDesc *objectDesc) override;
    void findMujocoObjectFromPrimPath();
    void CARB_ABI convertAndInitializeMujoco();
    // specific reset to this simulator
    void reset();
    void onDefaultUsdStageChanged(long stageId) override;
    void onTimelineEvent(omni::timeline::TimelineEventType timelineEventType);

    void CARB_ABI onAttach(long int stageId, double metersPerUnit) {
        std::cout << "OmniMuJoCoInterface::onAttach" << std::endl;
        mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        dataManager.mStageId.id = stageId;
        reset();
    }

    // detach the stage
    void CARB_ABI onDetach() {
        std::cout << "OmniMuJoCoInterface::onDetach" << std::endl;
        mStage = nullptr;
        reset();
    }

    // simulation was paused
    void CARB_ABI onPause() {
        std::cout << "OmniMuJoCoInterface::onPause" << std::endl;
        mPaused = true;
        reset();
    }

    // simulation was stopped(reset)
    void CARB_ABI onStop() {
        std::cout << "OmniMuJoCoInterface::onStop" << std::endl;
        dataManager.updateUsd(mMujoco, mUsdData, mStage, mStageInitialized, 0, 0);
        reset();
    }

    // simulation update
    void CARB_ABI onUpdate(float currentTime, float elapsedSecs) {
        // dataManager.updateUsd(mMujoco, mUsdData, mStage, mStageInitialized, currentTime, elapsedSecs);
        dataManager.updateFabric(mMujoco, mUsdData, mStage, mStageInitialized, currentTime, elapsedSecs);
    }

    // simulation was resumed
    void CARB_ABI onResume(float currentTime);
    /*
        The following call backs are called when there is change in usd scene (e.g. user interaction, scripting, )
        Each plugin should query the usd stage based on provided prim path name and sync changes related to their
    */
    // this gets called when a new Usd prim was added to the scene
    void CARB_ABI onPrimAdd(const pxr::SdfPath &primPath) {
        std::cout << "OmniMuJoCoInterface::onPrimAdd" << std::endl;
    }

    // this gets called when some properties in the usd prim was changed (e.g. manipulator or script changes transform)
    void CARB_ABI onPrimOrPropertyChange(const pxr::SdfPath &primOrPropertyPath) {
        std::cout << "OmniMuJoCoInterface::onPrimOrPropertyChange" << std::endl;
    }

    // this gets called when the named usd prim was removed from the scene
    void CARB_ABI onPrimRemove(const pxr::SdfPath &primPath) {
        std::cout << "OmniMuJoCoInterface::onPrimRemove" << std::endl;
    }

    /**
     * Temporary raycast handler.  This will become part of a more general user event handler.
     *
     * @param orig the ray origin.  Set to NULL to send a stop command for grabbing.
     * @param dir the ray direction (should be normalized).
     * @param input whether the input control is set or reset (e.g. mouse down).
     */
    void CARB_ABI onRaycast(const float *orig, const float *dir, bool input) {
        std::cout << "OmniMuJoCoInterface::onRaycast" << std::endl;
    }

    void onObjectsChanged(const UsdNotice::ObjectsChanged &objectsChanged) {
        std::cout << "OmniMuJoCoInterface::onObjectsChanged" << std::endl;
        // Check whether any of the prims we created have been (potentially) invalidated.
        // This may be too broad a check, but handles prims being removed from the stage.
    }
};
} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni