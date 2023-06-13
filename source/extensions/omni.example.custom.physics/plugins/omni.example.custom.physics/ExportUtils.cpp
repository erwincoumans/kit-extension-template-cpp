// Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#pragma once

#include "ExportUtils.h"
#include <stack>

#pragma warning(disable : 4996)

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
    strcpy_s(fullPath + baseLength, strSize + 1, fileRelativePath);
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
    auto desc = usdData.rigidBodyMap[childPath];

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
                                     const omni::physics::schema::ShapeDesc *shapeDesc) {
    tinyxml2::XMLElement *mOrigin = xmlDoc.NewElement("origin");
    mOrigin->SetAttribute("xyz", Vec3ToStr(shapeDesc->localPos * metersPerUnit).c_str());
    mOrigin->SetAttribute("quat_xyzw", QuatToStr(shapeDesc->localRot).c_str());
    xmlElement->InsertEndChild(mOrigin);

    tinyxml2::XMLElement *mGeometry = xmlDoc.NewElement("geometry");
    xmlElement->InsertEndChild(mGeometry);

    switch (shapeDesc->type) {
    case omni::physics::schema::ObjectType::eSphereShape: {
        auto desc = static_cast<const omni::physics::schema::SphereShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("sphere");
        mGeom->SetAttribute("radius ", desc->radius * metersPerUnit);
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCubeShape: {
        auto desc = static_cast<const omni::physics::schema::CubeShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("box");
        mGeom->SetAttribute("size", Vec3ToStr(2 * desc->halfExtents * metersPerUnit).c_str());
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCylinderShape: {
        auto desc = static_cast<const omni::physics::schema::CylinderShapeDesc *>(shapeDesc);
        tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("cylinder");
        mGeom->SetAttribute("radius", desc->radius * metersPerUnit);
        mGeom->SetAttribute("length", 2 * desc->halfHeight * metersPerUnit);
        mGeometry->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCapsuleShape:
    case omni::physics::schema::ObjectType::eConeShape:
    case omni::physics::schema::ObjectType::eMeshShape:
    case omni::physics::schema::ObjectType::eCustomShape:
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
    auto shapeDesc = usdData.shapeMap[childPath];
    shapeDescToUrdfXml(xmlDoc, mCollision, shapeDesc);
    parent->InsertEndChild(mCollision);
}

void ExportUtils::addUrdfVisual(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                const pxr::SdfPath &childPath) {

    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    tinyxml2::XMLElement *mVisual = xmlDoc.NewElement("visual");
    auto shapeDesc = usdData.shapeMap[childPath];
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
                                     const pxr::SdfPath &childPath, const omni::physics::schema::ShapeDesc *shapeDesc) {

    tinyxml2::XMLElement *mGeom = xmlDoc.NewElement("geom");
    pxr::UsdPrim child = stage->GetPrimAtPath(childPath);
    std::cout << "\t shape : " << childPath << std::endl;
    mGeom->SetAttribute("name", child.GetName().GetText());
    mGeom->SetAttribute("pos", Vec3ToStr(shapeDesc->localPos * metersPerUnit).c_str());
    mGeom->SetAttribute("quat", QuatToStr(shapeDesc->localRot).c_str());
    switch (shapeDesc->type) {
    case omni::physics::schema::ObjectType::eSphereShape: {
        auto desc = static_cast<const omni::physics::schema::SphereShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "sphere");
        mGeom->SetAttribute("size ", desc->radius * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCubeShape: {
        auto desc = static_cast<const omni::physics::schema::CubeShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "box");
        mGeom->SetAttribute("size",
                            Vec3ToStr(GfCompMult(desc->halfExtents, shapeDesc->localScale) * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCylinderShape: {
        auto desc = static_cast<const omni::physics::schema::CylinderShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "cylinder");
        std::stringstream ss;
        ss << 2 * desc->halfHeight * metersPerUnit << " " << desc->radius * metersPerUnit; // TODO: use the localScale
        mGeom->SetAttribute("size", desc->halfHeight * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eCapsuleShape: {
        auto desc = static_cast<const omni::physics::schema::CapsuleShapeDesc *>(shapeDesc);
        mGeom->SetAttribute("type", "capsule");
        std::stringstream ss;
        ss << 2 * desc->halfHeight * metersPerUnit << " " << desc->radius * metersPerUnit;
        mGeom->SetAttribute("size", 2 * desc->halfHeight * metersPerUnit);
        xmlElement->InsertEndChild(mGeom);
        break;
    }
    case omni::physics::schema::ObjectType::eConeShape:
    case omni::physics::schema::ObjectType::eMeshShape:

    case omni::physics::schema::ObjectType::eCustomShape:
        CARB_LOG_WARN("unsupported shape encountered in ExportUtils::shapeDescToMjcfXml");
        break;
    }
}

void ExportUtils::JointDescToMjcfXml(tinyxml2::XMLDocument& xmlDoc, tinyxml2::XMLElement* xmlElement,
	const pxr::SdfPath& primPath, const pxr::SdfPath& parentPath, const omni::physics::schema::JointDesc* jointDesc) {

    tinyxml2::XMLElement *mJoint = xmlDoc.NewElement("joint");
    pxr::UsdPrim prim = stage->GetPrimAtPath(primPath);
    std::cout << "\t joint : " << primPath << std::endl;
    std::string parentName = stage->GetPrimAtPath(parentPath).GetName().GetText();
    mJoint->SetAttribute("name", prim.GetName().GetText());
    auto getLocalAxis = [](pxr::GfQuatf q, omni::physics::schema::Axis::Enum axis) {
        switch (axis) {
        case omni::physics::schema::Axis::Enum::eX:
            return pxr::GfRotation(q).TransformDir(pxr::GfVec3f(1, 0, 0));
        case omni::physics::schema::Axis::Enum::eY:
            return pxr::GfRotation(q).TransformDir(pxr::GfVec3f(0, 1, 0));
        case omni::physics::schema::Axis::Enum::eZ:
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

    switch (jointDesc->type) {
    case omni::physics::schema::ObjectType::eJointRevolute: {
        auto desc = static_cast<const omni::physics::schema::RevoluteJointDesc *>(jointDesc);
        mJoint->SetAttribute("type", "hinge");
        mJoint->SetAttribute("axis", Vec3ToStr(getLocalAxis(jointDesc->localPose0Orientation, desc->axis)).c_str());
        mJoint->SetAttribute("pos", Vec3ToStr(localPosePosition * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mJoint);
        break;
    }
    case omni::physics::schema::ObjectType::eJointPrismatic: {
        auto desc = static_cast<const omni::physics::schema::PrismaticJointDesc *>(jointDesc);
        mJoint->SetAttribute("type", "slide");
        mJoint->SetAttribute("axis", Vec3ToStr(getLocalAxis(jointDesc->localPose0Orientation, desc->axis)).c_str());
        mJoint->SetAttribute("pos", Vec3ToStr(localPosePosition * metersPerUnit).c_str());
        xmlElement->InsertEndChild(mJoint);
        break;
    }

    default:
        CARB_LOG_WARN("unsupported joint encountered in ExportUtils::JointDescToMjcfXml ");
        break;
    }
}

tinyxml2::XMLElement* ExportUtils::addMjcfLink(tinyxml2::XMLDocument& xmlDoc, tinyxml2::XMLElement* parent,
	const pxr::SdfPath& childPath) {
	auto desc = usdData.rigidBodyMap[childPath];
	pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
	tinyxml2::XMLElement* mlink = xmlDoc.NewElement("body");
	mlink->SetAttribute("name", prim.GetName().GetText());
	mlink->SetAttribute("pos", Vec3ToStr(desc->position * metersPerUnit).c_str());
	mlink->SetAttribute("quat", QuatToStr(desc->rotation).c_str());
	auto shapeDesc = usdData.shapeMap[childPath];
	shapeDescToMjcfXml(xmlDoc, mlink, childPath, shapeDesc);
	if (desc->rigidBodyEnabled && !desc->kinematicBody && !usdData.visited[childPath]) {
		tinyxml2::XMLElement* mJoint = xmlDoc.NewElement("freejoint");
		mlink->InsertEndChild(mJoint);
	}
	parent->InsertEndChild(mlink);

	return mlink;
}

void ExportUtils::addMjcfShapes(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                const pxr::SdfPath &childPath) {
    pxr::UsdPrim prim = stage->GetPrimAtPath(childPath);
    std::cout << "processing shape " << prim.GetName().GetText() << std::endl;
    auto shapeDesc = usdData.shapeMap[childPath];
    shapeDescToMjcfXml(xmlDoc, parent, childPath, shapeDesc);
}

void ExportUtils::ExportToMjcf(const std::string fullPath) {
    std::string filePath = fullPath;
    char relPathBuffer[2048];
    MakeRelativePath(filePath.c_str(), "", relPathBuffer, filePath.size());
    metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);

    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLElement *pRoot = xmlDoc.NewElement("mujoco");

    std::cout << " UrdfExporter usdData.bodyPrims.size() = " << usdData.bodyPrims.size() << std::endl;
    buildKinematicTree(xmlDoc, pRoot);
    xmlDoc.InsertFirstChild(pRoot);


    // rigid bodies
    tinyxml2::XMLElement *wBody = xmlDoc.NewElement("worldbody");
    // for (size_t i = 0; i < usdData.shapePrims.size(); i++) {
    //     std::cout << " eval shape " << usdData.shapePrims[i].GetPath() << std::endl;
    //     if (usdData.rigidBodyMap.find(usdData.shapePrims[i].GetPath()) == usdData.rigidBodyMap.end()){
    //         std::cout << " processing shape " << usdData.shapePrims[i].GetPath() << std::endl;
    //         addMjcfShapes(xmlDoc, wBody, usdData.shapePrims[i].GetPath());
    //     }
    // }

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

void ExportUtils::buildKinematicTree(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement* pRoot) {

    for (auto it = usdData.articulationMap.begin(); it != usdData.articulationMap.end(); it++) {

        tinyxml2::XMLElement *wBody = xmlDoc.NewElement("worldbody");
        auto root = it->first;
        std::stack<pxr::SdfPath> rootStack;
        rootStack.push(root);
        usdData.visited[root] = true;
		tinyxml2::XMLElement* currentHead = addMjcfLink(xmlDoc, wBody, root);
        while (!rootStack.empty()) {
            auto path = rootStack.top();
            usdData.visited[path] = true;
            std::string bodyName = stage->GetPrimAtPath(path).GetName().GetText();
            std::cout << "visiting " << bodyName << std::endl;
            rootStack.pop();
            for (int i = 0; i < usdData.bodyGraph[path].size(); i++) {
                auto childBodyPath = usdData.bodyGraph[path][i].first;
                auto connectingJoint = usdData.bodyGraph[path][i].second;
                std::string childName = stage->GetPrimAtPath(childBodyPath).GetName().GetText();
                std::cout << "\t child : " << childName << std::endl;
                if (!usdData.visited[childBodyPath]) {

                    usdData.visited[childBodyPath] = true;
                    auto bodyDesc = usdData.rigidBodyMap[childBodyPath];
                    currentHead = addMjcfLink(xmlDoc, currentHead, childBodyPath);
                    JointDescToMjcfXml(xmlDoc, currentHead, connectingJoint, childBodyPath, usdData.jointMap[connectingJoint]);
                    rootStack.push(childBodyPath);
                }
            }
        }

        pRoot->InsertEndChild(wBody);


    }
}

ExportUtils::ExportUtils(pxr::UsdStageWeakPtr stage, UsdData &data) : stage(stage), usdData(data) {
    metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stage);
}

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
