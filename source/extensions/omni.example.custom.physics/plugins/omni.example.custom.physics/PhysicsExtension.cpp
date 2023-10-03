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

#define CARB_EXPORTS
#define MAX_MJ_ERROR_LENGTH 500

#include "OmniMuJoCoUpdateNode.h"
#include <omni/example/custom/physics/ICustomPhysicsInterface.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

using namespace carb;
using namespace omni::example::custom::physics;

OmniMujocoUpdateNode *gOmniMujocoUpdateNode = nullptr;
const omni::fabric::IPath *omni::fabric::Path::iPath = nullptr;
const omni::fabric::IToken *omni::fabric::Token::iToken = nullptr;

class pluginEvents {
  public:
    static void CARB_ABI onAttach(long int stageId, double metersPerUnit, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onAttach(stageId, metersPerUnit);
    }

    // detach the stage
    static void CARB_ABI onDetach(void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onDetach();
    }

    // simulation was paused
    static void CARB_ABI onPause(void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onPause();
    }

    // simulation was stopped(reset)
    static void CARB_ABI onStop(void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onStop();
    }

    // simulation was resumed
    static void CARB_ABI onResume(float currentTime, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onResume(currentTime);
    }

    static void CARB_ABI onUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings *settings,
                                  void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onUpdate(currentTime, elapsedSecs);
    }

    static void CARB_ABI onPrimAdd(const pxr::SdfPath &primPath, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onPrimAdd(primPath);
    }

    static void CARB_ABI onRaycast(const float *orig, const float *dir, bool input, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onRaycast(orig, dir, input);
    }

    // this gets called when some properties in the usd prim was changed (e.g. manipulator or script changes transform)
    static void CARB_ABI onPrimOrPropertyChange(const pxr::SdfPath &primOrPropertyPath, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onPrimOrPropertyChange(primOrPropertyPath);
    }

    // this gets called when the named usd prim was removed from the scene
    static void CARB_ABI onPrimRemove(const pxr::SdfPath &primPath, void *userData) {
        assert(usedData);
        OmniMujocoUpdateNode *node = (OmniMujocoUpdateNode *)userData;
        node->onPrimRemove(primPath);
    }
};

CARB_EXPORT void carbOnPluginStartup() {
    std::cout << "carbOnPluginStartup" << std::endl;
    CARB_LOG_WARN("carbOnPluginStartup physics.");
    carb::Framework *framework = carb::getFramework();
    auto iStageUpdate = framework->tryAcquireInterface<omni::kit::IStageUpdate>();
    {
        omni::kit::StageUpdateNodeDesc desc = {0};
        desc.displayName = "Custom Physics";
        desc.order = omni::kit::update::eIUsdStageUpdatePhysics;
        gOmniMujocoUpdateNode = new OmniMujocoUpdateNode();
        desc.userData = gOmniMujocoUpdateNode;
        desc.onAttach = pluginEvents::onAttach;
        desc.onDetach = pluginEvents::onDetach;
        desc.onUpdate = pluginEvents::onUpdate;
        desc.onResume = pluginEvents::onResume;
        desc.onPause = pluginEvents::onPause;
        desc.onStop = pluginEvents::onStop;
        desc.onRaycast = nullptr;

        gOmniMujocoUpdateNode->mStageUpdateNode = iStageUpdate->createStageUpdateNode(desc);
    }
}

CARB_EXPORT void carbOnPluginShutdown() {
    carb::Framework *framework = carb::getFramework();
    auto iStageUpdate = framework->tryAcquireInterface<omni::kit::IStageUpdate>();

    if (gOmniMujocoUpdateNode != nullptr) {
        iStageUpdate->destroyStageUpdateNode(gOmniMujocoUpdateNode->mStageUpdateNode);
        delete gOmniMujocoUpdateNode;
        gOmniMujocoUpdateNode = nullptr;
    }
}

const struct carb::PluginImplDesc pluginImplDesc = {"omni.example.custom.physics.plugin", "An example C++ extension.",
                                                    "NVIDIA", carb::PluginHotReload::eEnabled, "dev"};

CARB_PLUGIN_IMPL(pluginImplDesc, OmniMujocoUpdateNode)
CARB_PLUGIN_IMPL_DEPS(omni::physics::schema::IUsdPhysics)
void fillInterface(OmniMujocoUpdateNode &iface) {}

// CARB_PLUGIN_IMPL(pluginImplDesc, ICustomPhysicsInterface)

// CARB_PLUGIN_IMPL(pluginImplDesc, omni::example::custom::physics::OmniMujocoUpdateNode)
// CARB_PLUGIN_IMPL_DEPS(omni::physics::schema::IUsdPhysics)
// void fillInterface(omni::example::custom::physics::OmniMujocoUpdateNode &iface) {}
