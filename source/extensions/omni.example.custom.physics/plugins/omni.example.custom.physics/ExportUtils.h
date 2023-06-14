// Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#pragma once

#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <tinyxml2.h>

#pragma warning(push)
#pragma warning(disable : 4267)
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <omni/usd/UtilsIncludes.h>
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
#include <pxr/usd/usdUtils/stageCache.h>
#include <usdPhysics/massAPI.h>

#include <omni/physics/schema/IUsdPhysics.h>
#include <omni/physics/schema/IUsdPhysicsListener.h>

#pragma warning(pop)
// normal code

using ArticulationMap = std::map<pxr::SdfPath, const omni::physics::schema::ArticulationDesc *>;
using RigidBodyMap = std::map<pxr::SdfPath, const omni::physics::schema::RigidBodyDesc *>;
using ShapeMap = std::map<pxr::SdfPath, const omni::physics::schema::ShapeDesc *>;
using ShapeScale = std::map<pxr::SdfPath, pxr::GfVec3f>;
using JointMap = std::map<pxr::SdfPath, const omni::physics::schema::JointDesc *>;
using BodyGraph = std::map<pxr::SdfPath, std::vector<std::pair<pxr::SdfPath, pxr::SdfPath>>>;
using VisitedBodies = std::map<pxr::SdfPath, bool>;

namespace omni {
namespace example {
namespace custom {
namespace physics {

class UsdData {
  public:
    omni::physics::schema::SceneDesc sceneDesc;
    pxr::SdfPath scenePath;
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
    }
};

class ExportUtils {
  public:
    pxr::UsdStageWeakPtr stage;
    UsdData &usdData;
    double metersPerUnit = 1.0;
    std::string baseDirPath;
    bool isLoaded = false;
    ExportUtils(pxr::UsdStageWeakPtr stage, UsdData &data);
    void ExportToUrdf(const std::string fullPath);
    void ExportToMjcf(const std::string fullPath);
    void addUrdfInertial(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfCollision(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfVisual(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void shapeDescToUrdfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                            const omni::physics::schema::ShapeDesc *shapeDesc);
    void shapeDescToMjcfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                            const pxr::SdfPath &childPath, const omni::physics::schema::ShapeDesc *shapeDesc);
    tinyxml2::XMLElement *addMjcfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                      const pxr::SdfPath &childPath, const pxr::SdfPath &parentPath = pxr::SdfPath());
    void addMjcfShapes(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void JointDescToMjcfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                            const pxr::SdfPath &primPath, const pxr::SdfPath &parentPath,
                            const omni::physics::schema::JointDesc *jointDesc);
    void buildKinematicTree(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *pRoot);
};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
