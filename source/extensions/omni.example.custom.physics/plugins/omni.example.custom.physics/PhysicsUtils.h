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
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>

#include <mujoco/mujoco.h>

#include <carb/Defines.h>
#include <carb/PluginUtils.h>
// legacy code
#include "pxr/usd/usd/collectionMembershipQuery.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdUtils/stageCache.h>

#pragma warning(push)
#pragma warning(disable : 4267)
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <omni/physics/schema/IUsdPhysics.h>
#include <omni/physics/schema/IUsdPhysicsListener.h>
#include <omni/usd/UtilsIncludes.h>
#pragma warning(pop)

namespace omni {
namespace example {
namespace custom {
namespace physics {

using ArticulationMap = std::map<pxr::SdfPath, omni::physics::schema::ArticulationDesc *>;
using RigidBodyMap = std::map<pxr::SdfPath, omni::physics::schema::RigidBodyDesc *>;
using ShapeMap = std::map<pxr::SdfPath, omni::physics::schema::ShapeDesc *>;
using JointMap = std::map<pxr::SdfPath, omni::physics::schema::JointDesc *>;
using BodyGraph = std::map<pxr::SdfPath, std::vector<std::pair<pxr::SdfPath, pxr::SdfPath>>>;
using VisitedBodies = std::map<pxr::SdfPath, bool>;
using ShapeScale = std::map<pxr::SdfPath, pxr::GfVec3f>;

class MujocoData {
  public:
    mjvScene scene;
    mjrContext context;
    mjvPerturb perturb;
    mjvCamera camera;
    mjvOption option;
    mjModel *model;
    mjData *data;
    std::map<int, pxr::SdfPath> idsToPaths;
    std::map<pxr::SdfPath, int> pathsToIds;
    void reset() {
        idsToPaths.clear();
        pathsToIds.clear();
    }
};

class UsdData {
  public:
    omni::physics::schema::SceneDesc sceneDesc;
    pxr::SdfPath scenePath;
    std::vector<omni::physics::schema::ObjectDesc *> allDesc;
    std::vector<pxr::UsdPrim> bodyPrims;
    std::vector<pxr::UsdPrim> shapePrims;
    RigidBodyMap rigidBodyMap;
    ArticulationMap articulationMap;
    JointMap jointMap;
    ShapeMap shapeMap;
    ShapeScale shapeScales;
    BodyGraph bodyGraph;
    VisitedBodies visited;
    void reset() {
        bodyPrims.clear();
        shapePrims.clear();
        rigidBodyMap.clear();
        articulationMap.clear();
        jointMap.clear();
        shapeMap.clear();
        visited.clear();
        bodyGraph.clear();
        shapePrims.clear();
        scenePath = pxr::SdfPath();
    }
    ~UsdData() {
        for (auto i = 0; i < allDesc.size(); i++) {
            delete allDesc[i];
        }
        allDesc.clear();
    }
};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni