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

#include "ExportUtils.h"
#include <cstring>
#include <stack>

namespace OPS = omni::physics::schema;

namespace omni {
namespace example {
namespace custom {
namespace physics {

// given the path of one file it strips the filename and appends the relative path onto it
static void MakeRelativePath(const char *filePath, const char *fileRelativePath, char *fullPath, size_t strSize) {
    // get base path of file
    const char *lastSlash = nullptr;

    if (!lastSlash)
        lastSlash = strrchr(filePath, '\\');
    if (!lastSlash)
        lastSlash = strrchr(filePath, '/');

    int baseLength = 0;

    if (lastSlash) {
        baseLength = int(lastSlash - filePath) + 1;

        // copy base path (including slash to relative path)
        memcpy(fullPath, filePath, baseLength);
    }

    // if (fileRelativePath[0] == '.')
    //++fileRelativePath;
    if (fileRelativePath[0] == '\\' || fileRelativePath[0] == '/')
        ++fileRelativePath;

    // append mesh filename
#if _WIN32
    strcpy_s(fullPath + baseLength, strSize + 1, fileRelativePath);
#else
    strcpy(fullPath, fileRelativePath);
#endif

}

static inline std::string Vec3ToStr(const pxr::GfVec3f v) {
    std::stringstream ss;
    ss << v[0] << " " << v[1] << " " << v[2];
    return ss.str();
}

static inline std::string QuatToStr(const pxr::GfQuatf v) {
    std::stringstream ss;
    ss << v.GetReal() << " " << v.GetImaginary()[0] << " " << v.GetImaginary()[1] << " " << v.GetImaginary()[2];
    return ss.str();
}

void ExportUtils::addUrdfInertial(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                  const pxr::SdfPath &childPath) {

    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mInertial = xmlDoc.NewElement("inertial");
    parent->InsertEndChild(mInertial);
    const OPS::RigidBodyDesc *desc = usdData.rigidBodyMap[childPath];

    tinyxml2::XMLElement *mOrigin =
        xmlDoc.NewElement("origin"); // TODO: this should be the CMassLocalPose not the pose of the link
    mOrigin->SetAttribute("xyz", Vec3ToStr(desc->position * metersPerUnit).c_str());
    mOrigin->SetAttribute("quat_xyzw", QuatToStr(desc->rotation).c_str());
    mInertial->InsertEndChild(mOrigin);

    float mass(0);
    pxr::GfVec3f inertia(0);
    const pxr::UsdPhysicsMassAPI api = pxr::UsdPhysicsMassAPI::Get(stage, childPath);
    if (api) {
        tinyxml2::XMLElement *mMass = xmlDoc.NewElement("mass");
        tinyxml2::XMLElement *mInertia = xmlDoc.NewElement("inertia");
        api.GetMassAttr().Get(&mass);
        api.GetDiagonalInertiaAttr().Get(&inertia);
        mMass->SetAttribute("value", mass);
        mInertia->SetAttribute("ixx", inertia[0]);
        mInertia->SetAttribute("iyy", inertia[1]);
        mInertia->SetAttribute("izz", inertia[2]);
        mInertia->SetAttribute("ixy", 0.0); // TODO: how to get this from USD
        mInertia->SetAttribute("ixz", 0.0);
        mInertia->SetAttribute("iyz", 0.0);
        mInertial->InsertEndChild(mMass);
        mInertial->InsertEndChild(mInertia);
    }
}
void ExportUtils::shapeDescToUrdfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                                     const OPS::ShapeDesc *shapeDesc) {
    tinyxml2::XMLElement *mOrigin = xmlDoc.NewElement("origin");
    mOrigin->SetAttribute("xyz", Vec3ToStr(shapeDesc->localPos * metersPerUnit).c_str());
    mOrigin->SetAttribute("quat_xyzw", QuatToStr(shapeDesc->localRot).c_str());
    xmlElement->InsertEndChild(mOrigin);

    tinyxml2::XMLElement *mGeometry = xmlDoc.NewElement("geometry");
    xmlElement->InsertEndChild(mGeometry);

    switch (shapeDesc->type) {
    case OPS::ObjectType::eSphereShape: {
        auto desc = static_cast<const OPS::SphereShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("sphere");
        mGeom->SetAttribute("radius ", desc->radius * metersPerUnit);
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCubeShape: {
        auto desc = static_cast<const OPS::CubeShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("box");
        mGeom->SetAttribute("size", Vec3ToStr(2 * desc->halfExtents * metersPerUnit).c_str());
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCylinderShape: {
        auto desc = static_cast<const OPS::CylinderShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("cylinder");
        mGeom->SetAttribute("radius", desc->radius * metersPerUnit);
        mGeom->SetAttribute("length", 2 * desc->halfHeight * metersPerUnit);
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCapsuleShape:
    case OPS::ObjectType::eConeShape:
    case OPS::ObjectType::eMeshShape:
    case OPS::ObjectType::eCustomShape:
        CARB_LOG_WARN("unsupported shape encountered in UrdfExporter::shapeDescToXml ");
        break;
    }
}
void ExportUtils::addUrdfCollision(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                   const pxr::SdfPath &childPath) {

    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mCollision = xmlDoc.NewElement("collision");
    // auto collisionsPaths  = usdData.rigidBodyMap[childPath]->collisions; //collision set
    // for (auto path : collisionsPaths){
    //     auto shapeDesc  = usdData.shapeMap[path]; //collision set
    //     if(shapeDesc->collisionEnabled){
    //         shapeDescToXml(xmlDoc, mCollision, shapeDesc);
    //     }
    // }
    const OPS::ShapeDesc *shapeDesc = usdData.shapeMap[childPath];
    shapeDescToUrdfXml(xmlDoc, mCollision, shapeDesc);
    parent->InsertEndChild(mCollision);
}

void ExportUtils::addUrdfVisual(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                const pxr::SdfPath &childPath) {

    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mVisual = xmlDoc.NewElement("visual");
    const OPS::ShapeDesc *shapeDesc = usdData.shapeMap[childPath];
    shapeDescToUrdfXml(xmlDoc, mVisual, shapeDesc);
    parent->InsertEndChild(mVisual);
}

void ExportUtils::addUrdfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                              const pxr::SdfPath &childPath) {
    pxr::UsdPrim child = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mlink = xmlDoc.NewElement("link");
    mlink->SetAttribute("name", child.GetName().GetText());

    addUrdfInertial(xmlDoc, mlink, childPath);
    addUrdfVisual(xmlDoc, mlink, childPath);
    addUrdfCollision(xmlDoc, mlink, childPath);
    parent->InsertEndChild(mlink);
}

void ExportUtils::ExportToUrdf(const std::string fullPath) {

    std::string filePath = fullPath;
    char relPathBuffer[2048];
    MakeRelativePath(filePath.c_str(), "", relPathBuffer, filePath.size());
    metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);

    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLElement *pRoot = xmlDoc.NewElement("robot");
    pRoot->SetAttribute("name", "");

    std::cout << " UrdfExporter data.bodyPrims.size() = " << usdData.bodyPrims.size() << std::endl;
    for (size_t i = 0; i < usdData.bodyPrims.size(); i++) {
        addUrdfLink(xmlDoc, pRoot, usdData.bodyPrims[i].GetPath());
    }
    xmlDoc.InsertFirstChild(pRoot);
    tinyxml2::XMLPrinter printer;
    xmlDoc.Print(&printer);
    std::string xmlFile = printer.CStr();
    xmlDoc.SaveFile(filePath.c_str());
}

void ExportUtils::shapeDescToMjcfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                                     const pxr::SdfPath &childPath, const OPS::ShapeDesc *shapeDesc) {

    tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("geom");
    pxr::UsdPrim child = stage->GetPrimAtPath(childPath);
    std::cout << "\t shape : " << childPath << std::endl;
    mGeom->SetAttribute("name", childPath.GetText());
    mGeom->SetAttribute("pos", Vec3ToStr(shapeDesc->localPos * metersPerUnit).c_str());
    mGeom->SetAttribute("quat", QuatToStr(shapeDesc->localRot).c_str());
    switch (shapeDesc->type) {
    case OPS::ObjectType::eSphereShape: {
        auto desc = static_cast<const OPS::SphereShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "sphere");
        mGeom->SetAttribute("size ", desc->radius * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCubeShape: {
        auto desc = static_cast<const OPS::CubeShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "box");
        mGeom->SetAttribute("size",
                            Vec3ToStr(GfCompMult(desc->halfExtents, shapeDesc->localScale) * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCylinderShape: {
        auto desc = static_cast<const OPS::CylinderShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "cylinder");
        std::stringstream ss;
        ss << 2 * desc->halfHeight * metersPerUnit << " " << desc->radius * metersPerUnit; // TODO: use the localScale
        mGeom->SetAttribute("size", desc->halfHeight * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eCapsuleShape: {
        auto desc = static_cast<const OPS::CapsuleShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "capsule");
        std::stringstream ss;
        ss << 2 * desc->halfHeight * metersPerUnit << " " << desc->radius * metersPerUnit;
        mGeom->SetAttribute("size", 2 * desc->halfHeight * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case OPS::ObjectType::eConeShape:
    case OPS::ObjectType::eMeshShape:

    case OPS::ObjectType::eCustomShape:
        CARB_LOG_WARN("unsupported shape encountered in ExportUtils::shapeDescToMjcfXml");
        break;
    }
}

void ExportUtils::addMjcfJoint(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                               const pxr::SdfPath &primPath, const pxr::SdfPath &parentPath) {

    tinyxml2::XMLElement *mJoint = xmlDoc.NewElement("joint");
    pxr::UsdPrim prim = stage->GetPrimAtPath(primPath);
    std::string parentName = stage->GetPrimAtPath(parentPath).GetName().GetText();
    std::string name = primPath.GetText();
    mJoint->SetAttribute("name", name.c_str());
    const OPS::JointDesc *jointDesc = usdData.jointMap[primPath];
    auto getLocalAxis = [](pxr::GfQuatf q, OPS::Axis::Enum axis) {
        switch (axis) {
        case OPS::Axis::Enum::eX:
            return pxr::GfRotation(q).TransformDir(pxr::GfVec3f(1, 0, 0));
        case OPS::Axis::Enum::eY:
            return pxr::GfRotation(q).TransformDir(pxr::GfVec3f(0, 1, 0));
        case OPS::Axis::Enum::eZ:
            return pxr::GfRotation(q).TransformDir(pxr::GfVec3f(0, 0, 1));
        default: {
            return pxr::GfVec3f(1, 0, 0);
        } break;
        }
    };

    pxr::GfVec3f localPosePosition;
    if (jointDesc->body0 == parentPath)
        localPosePosition = jointDesc->localPose0Position;
    else
        localPosePosition = jointDesc->localPose1Position;

    // std::cout << "\t joint " << primPath << " desc : " << jointDesc << " joint type " << jointDesc->type << std::endl;
    switch (jointDesc->type) {
    case OPS::ObjectType::eJointRevolute: {
        auto desc = static_cast<const OPS::RevoluteJointDesc *>(jointDesc);
        mJoint->SetAttribute("type", "hinge");
        mJoint->SetAttribute("axis", Vec3ToStr(getLocalAxis(jointDesc->localPose0Orientation, desc->axis)).c_str());
        mJoint->SetAttribute("pos", Vec3ToStr(localPosePosition * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mJoint);
    } break;
    case OPS::ObjectType::eJointPrismatic: {
        auto desc = static_cast<const OPS::PrismaticJointDesc *>(jointDesc);
        mJoint->SetAttribute("type", "slide");
        mJoint->SetAttribute("axis", Vec3ToStr(getLocalAxis(jointDesc->localPose0Orientation, desc->axis)).c_str());
        mJoint->SetAttribute("pos", Vec3ToStr(localPosePosition * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mJoint);
    } break;
    case OPS::ObjectType::eJointFixed: {
        auto desc = static_cast<const OPS::FixedJointDesc *>(jointDesc);
        CARB_LOG_WARN("unsupported eJointFixed encountered in ExportUtils::addMjcfJoint");
    } break;
    case OPS::ObjectType::eJointSpherical: {
        auto desc = static_cast<const OPS::SphericalJointDesc *>(jointDesc);
        CARB_LOG_WARN("unsupported eJointSpherical encountered in ExportUtils::addMjcfJoint");
    } break;
    case OPS::ObjectType::eJointD6: {
        auto desc = static_cast<const OPS::D6JointDesc *>(jointDesc);
        CARB_LOG_WARN("unsupported eJointD6 encountered in ExportUtils::addMjcfJoint");
    } break;
    case OPS::ObjectType::eJointCustom: {
        auto desc = static_cast<const OPS::CustomJointDesc *>(jointDesc);
        CARB_LOG_WARN("unsupported eJointCustom encountered in ExportUtils::addMjcfJoint");
    } break;
    default:
        CARB_LOG_WARN("unsupported joint encountered in ExportUtils::addMjcfJoint");
        break;
    }
}

tinyxml2::XMLElement *ExportUtils::addMjcfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                               const pxr::SdfPath &childPath, const pxr::SdfPath &parentPath) {
    const OPS::RigidBodyDesc *desc = usdData.rigidBodyMap[childPath];
    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mlink = xmlDoc.NewElement("body");
    pxr::GfVec3f localPos;
    pxr::GfQuatf localRot;
    if (parentPath != pxr::SdfPath()) {
        pxr::UsdGeomXformCache xfCache;
        const pxr::GfMatrix4d matP = xfCache.GetLocalToWorldTransform(stage->GetPrimAtPath(parentPath));
        const pxr::GfMatrix4d matC = xfCache.GetLocalToWorldTransform(stage->GetPrimAtPath(childPath));
        pxr::GfTransform pTransform(matP);
        pxr::GfTransform cTransform(matC);
        // TODO: why the following doesn't work
        // const pxr::GfTransform localTrans(matC * matP.GetInverse());
        // localPos = (pxr::GfVec3f)localTrans.GetTranslation();
        // localRot = (pxr::GfQuatf)localTrans.GetRotation().GetQuat();
        localPos = (pxr::GfVec3f)(cTransform.GetTranslation() - pTransform.GetTranslation());
        localRot = (pxr::GfQuatf)(cTransform.GetRotation().GetQuat() * pTransform.GetRotation().GetQuat().GetInverse());
        // std::cout << cTransform.GetTranslation() << pTransform.GetTranslation()  << localTrans.GetTranslation()  <<
        // std::endl;

    } else {
        localPos = desc->position;
        localRot = desc->rotation;
    }
    mlink->SetAttribute("name", childPath.GetText());
    mlink->SetAttribute("pos", Vec3ToStr(localPos * metersPerUnit).c_str());
    mlink->SetAttribute("quat", QuatToStr(localRot).c_str());
    const OPS::ShapeDesc *shapeDesc = usdData.shapeMap[childPath];
    shapeDescToMjcfXml(xmlDoc, mlink, childPath, shapeDesc);
    if (desc->rigidBodyEnabled && !desc->kinematicBody && !usdData.visited[childPath]) {
        tinyxml2::XMLElement *mJoint = xmlDoc.NewElement("freejoint");
        mlink->InsertEndChild(mJoint);
    }
    parent->InsertEndChild(mlink);

    return mlink;
}

void ExportUtils::addMjcfShapes(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                const pxr::SdfPath &childPath) {
    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    std::cout << "processing shape " << prim.GetName().GetText() << std::endl;
    const OPS::ShapeDesc *shapeDesc = usdData.shapeMap[childPath];
    shapeDescToMjcfXml(xmlDoc, parent, childPath, shapeDesc);
}

void ExportUtils::buildKinematicTree(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *pRoot) {

    for (auto it = usdData.articulationMap.begin(); it != usdData.articulationMap.end(); it++) {

        tinyxml2::XMLElement *wBody = xmlDoc.NewElement("worldbody");
        pxr::SdfPath root = it->first;
        std::stack<pxr::SdfPath> rootStack;
        rootStack.push(root);
        usdData.visited[root] = true;
        tinyxml2::XMLElement *currentHead = addMjcfLink(xmlDoc, wBody, root);
        pxr::SdfPath parentPath = root;
        while (!rootStack.empty()) {
            pxr::SdfPath path = rootStack.top();
            usdData.visited[path] = true;
            std::string bodyName = stage->GetPrimAtPath(path).GetName().GetText();
            std::cout << "visiting " << path.GetText() << std::endl;
            rootStack.pop();
            for (auto i = 0; i < usdData.bodyGraph[path].size(); i++) {
                pxr::SdfPath childBodyPath = usdData.bodyGraph[path][i].first;
                pxr::SdfPath connectingJoint = usdData.bodyGraph[path][i].second;
                std::string childName = stage->GetPrimAtPath(childBodyPath).GetName().GetText();
                if (!usdData.visited[childBodyPath]) {
                    std::cout << "\t childBodyPath : " << childName << " connectingJoint : " << connectingJoint
                              << std::endl;
                    usdData.visited[childBodyPath] = true;
                    const OPS::RigidBodyDesc *bodyDesc = usdData.rigidBodyMap[childBodyPath];
                    currentHead = addMjcfLink(xmlDoc, currentHead, childBodyPath, parentPath);
                    addMjcfJoint(xmlDoc, currentHead, connectingJoint, childBodyPath);
                    rootStack.push(childBodyPath);
                    parentPath = childBodyPath;
                }
            }
        }

        pRoot->InsertEndChild(wBody);
    }
}

void ExportUtils::ExportToMjcf(const std::string fullPath) {
    std::string filePath = fullPath;
    char relPathBuffer[2048];
    MakeRelativePath(filePath.c_str(), "", relPathBuffer, filePath.size());
    metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);

    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLElement *pRoot = xmlDoc.NewElement("mujoco");

    for (auto it = usdData.jointMap.begin(); it != usdData.jointMap.end(); it++) {
        std::cout << "\t joint " << it->first << " desc : " << it->second << " joint type " << it->second->type
                  << std::endl;
    }

    buildKinematicTree(xmlDoc, pRoot);
    xmlDoc.InsertFirstChild(pRoot);

    // rigid bodies and shapes
    tinyxml2::XMLElement *wBody = xmlDoc.NewElement("worldbody");
    for (size_t i = 0; i < usdData.shapePrims.size(); i++) {
        if (usdData.rigidBodyMap.find(usdData.shapePrims[i].GetPath()) == usdData.rigidBodyMap.end()) {
            std::cout << " processing shape " << usdData.shapePrims[i].GetPath() << std::endl;
            addMjcfShapes(xmlDoc, wBody, usdData.shapePrims[i].GetPath());
        }
    }

    bool hasAtLeastOneBody = false;
    for (size_t i = 0; i < usdData.bodyPrims.size(); i++) {
        if (!usdData.visited[usdData.bodyPrims[i].GetPath()]) {
            addMjcfLink(xmlDoc, wBody, usdData.bodyPrims[i].GetPath());
            hasAtLeastOneBody = true;
        }
    }
    if (hasAtLeastOneBody)
        pRoot->InsertEndChild(wBody);

    xmlDoc.SaveFile(filePath.c_str());
}

ExportUtils::ExportUtils(pxr::UsdStageWeakPtr stage, UsdData &data) : stage(stage), usdData(data) {
    metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);
}

ExportUtils::~ExportUtils() {}

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
