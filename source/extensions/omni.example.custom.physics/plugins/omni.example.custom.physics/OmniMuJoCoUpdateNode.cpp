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
#include "ExportUtils.h"
#include <omni/usd/UsdUtils.h>

namespace omni {
namespace example {
namespace custom {
namespace physics {

// omni::physics::schema::IUsdPhysicsListener interface
void OmniMujocoUpdateNode::parsePrim(const pxr::UsdPrim &prim, omni::physics::schema::ObjectDesc *objectDesc, uint64_t,
                                     const pxr::TfTokenVector &appliedApis) {
    // std::cout << "parsePrim" << std::endl;
}

void OmniMujocoUpdateNode::reportObjectDesc(const pxr::SdfPath &path,
                                            const omni::physics::schema::ObjectDesc *objectDesc) {
    switch (objectDesc->type) {
    case omni::physics::schema::ObjectType::eScene: {
        const pxr::UsdPrim scenePrim = mStage->GetPrimAtPath(path);
        if (scenePrim) {
            // We process only scenes without any API or with PhysxSceneAPI
            const pxr::TfTokenVector &appliedApis = scenePrim.GetPrimTypeInfo().GetAppliedAPISchemas();
            if (!appliedApis.empty()) {
                static const pxr::TfToken gCustomPhysicsSceneAPIToken("CustomPhysicsAPI");
                for (auto &api : appliedApis) {
                    if (api == gCustomPhysicsSceneAPIToken) {
                        mUsdData.sceneDesc = *(const omni::physics::schema::SceneDesc *)objectDesc;
                        auto tmpDesc = new omni::physics::schema::SceneDesc(mUsdData.sceneDesc);
                        mUsdData.allDesc.push_back(tmpDesc);
                        mUsdData.scenePath = path;
                        std::cout << path.GetText() << " is a scene" << std::endl;
                    }
                }
            }
        }
        break;
    }
    case omni::physics::schema::ObjectType::eRigidBody: {
        const omni::physics::schema::RigidBodyDesc *inDesc = (const omni::physics::schema::RigidBodyDesc *)objectDesc;
        for (auto &ownerPath : inDesc->simulationOwners) {
            if (ownerPath == mUsdData.scenePath) {
                auto tmpDesc = new omni::physics::schema::RigidBodyDesc(*inDesc);
                mUsdData.allDesc.push_back(tmpDesc);
                mUsdData.rigidBodyMap[path] = tmpDesc;
                mUsdData.bodyPrims.push_back(mStage->GetPrimAtPath(path));
                std::cout << path.GetText() << " is a rigid body" << std::endl;

                // todo: convert USD data to MuJoCo / URDF. Once all is done, 'compile' the model
                // todo2: refactor MuJoCo to not use a global 'Model' and 'Data' but more fine-grained data that can
                // be incrementally updated.
            }
        }
        break;
    }

    case omni::physics::schema::ObjectType::eArticulation: {
        const omni::physics::schema::ArticulationDesc *inDesc =
            (const omni::physics::schema::ArticulationDesc *)objectDesc;
        std::cout << path.GetText() << " is an articulation" << std::endl;
        auto tmpDesc = new omni::physics::schema::ArticulationDesc(*inDesc);
        mUsdData.allDesc.push_back(tmpDesc);
        mUsdData.articulationMap[path] = tmpDesc;
        break;
    }

        omni::physics::schema::JointDesc *jointDesc;
    case omni::physics::schema::ObjectType::eJointSpherical:
        jointDesc = new omni::physics::schema::SphericalJointDesc(
            *(const omni::physics::schema::SphericalJointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointD6:
        jointDesc = new omni::physics::schema::D6JointDesc(*(const omni::physics::schema::D6JointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointDistance:
        jointDesc =
            new omni::physics::schema::DistanceJointDesc(*(const omni::physics::schema::DistanceJointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointFixed:
        jointDesc =
            new omni::physics::schema::FixedJointDesc(*(const omni::physics::schema::FixedJointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointPrismatic:
        jointDesc = new omni::physics::schema::PrismaticJointDesc(
            *(const omni::physics::schema::PrismaticJointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointRevolute:
        jointDesc =
            new omni::physics::schema::RevoluteJointDesc(*(const omni::physics::schema::RevoluteJointDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eJointCustom: {
    }
        {
            // std::cout << jointDesc << " joint type " << jointDesc->type << std::endl;
            if (jointDesc->jointEnabled) {
                // only a fixed joint can have a dangling body0/body1, we can ignore in mjcf
                if (jointDesc->body0 != SdfPath() && jointDesc->body1 != SdfPath()) {
                    mUsdData.bodyGraph[jointDesc->body0].push_back(std::make_pair(jointDesc->body1, path));
                    mUsdData.visited[jointDesc->body0] = false;
                    mUsdData.bodyGraph[jointDesc->body1].push_back(std::make_pair(jointDesc->body0, path));
                    mUsdData.visited[jointDesc->body1] = false;
                    // std::cout << "\t body0 : " << jointDesc->body0 << ", body1 : " << jointDesc->body1 << std::endl;
                }
            }
            mUsdData.allDesc.push_back(jointDesc);
            mUsdData.jointMap[path] = jointDesc;
            // TODO: check for correctness of the bodies, whether joint is enabled, etc.
            // Later check for repeated bodies and loops
            break;
        }

        omni::physics::schema::ShapeDesc *desc;
    case omni::physics::schema::ObjectType::eSphereShape:
        desc = new omni::physics::schema::SphereShapeDesc(*(const omni::physics::schema::SphereShapeDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eCubeShape:
        desc = new omni::physics::schema::CubeShapeDesc(*(const omni::physics::schema::CubeShapeDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eCapsuleShape:
        desc =
            new omni::physics::schema::CapsuleShapeDesc(*(const omni::physics::schema::CapsuleShapeDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eCylinderShape:
        desc =
            new omni::physics::schema::CylinderShapeDesc(*(const omni::physics::schema::CylinderShapeDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eConeShape:
        desc = new omni::physics::schema::ConeShapeDesc(*(const omni::physics::schema::ConeShapeDesc *)objectDesc);
    case omni::physics::schema::ObjectType::eMeshShape: {
    }
        // desc = new omni::physics::schema::MeshShapeDesc(*(const omni::physics::schema::MeshShapeDesc*)objectDesc);
    case omni::physics::schema::ObjectType::eCustomShape: {
    }
        {
            const omni::physics::schema::ShapeDesc *inDesc = (const omni::physics::schema::ShapeDesc *)objectDesc;
            mUsdData.allDesc.push_back(desc);
            mUsdData.shapeMap[path] = desc;
            mUsdData.shapePrims.push_back(mStage->GetPrimAtPath(path));
            pxr::UsdPrim prim = mStage->GetPrimAtPath(path);
            // std::cout << "reportObjectDesc shape " << prim.GetName().GetText() << std::endl;
            pxr::GfVec3f scale;
            prim.GetAttribute(TfToken("xformOp:scale")).Get(&scale);
            mScales.push_back(scale);
            mUsdData.shapeScales[path] = scale;
            break;
        }
    }
}

void OmniMujocoUpdateNode::findMujocoObjectFromPrimPath() {
    for (int i = 0; i < mMujoco.scene.ngeom; i++) {
        const char *geomName = mj_id2name(mMujoco.model, mjOBJ_GEOM, i);
        if (geomName) {
            if (mUsdData.rigidBodyMap.find(pxr::SdfPath(geomName)) != mUsdData.rigidBodyMap.end()) {
                mMujoco.idsToPaths[i] = pxr::SdfPath(geomName);
                mMujoco.pathsToIds[pxr::SdfPath(geomName)] = i;
            } else {
                mMujoco.idsToPaths[i] = pxr::SdfPath();
            }
        }
    }
}
void CARB_ABI OmniMujocoUpdateNode::convertAndInitializeMujoco() {
    assert(mMujoco.model);
    assert(mMujoco.data);
#if _WIN32
    char tmp_path[1024];
    GetTempPathA(1024, tmp_path);
    fileName = std::string(tmp_path) + "mujoco_scene";
#else
    fileName = "/tmp/mujoco_scene";
#endif
    std::cout << "fileName:" << fileName << std::endl;

    // std::cout << "exporting to urdf" << std::endl;
    omni::example::custom::physics::ExportUtils exporter(mStage, mUsdData);
    // exporter.ExportToUrdf(fileName + ".urdf");
    exporter.ExportToMjcf(fileName + ".xml");
    mjv_defaultScene(&mMujoco.scene);

    char error[MAX_MJ_ERROR_LENGTH] = "";

    mMujoco.model = mj_loadXML((fileName + ".xml").c_str(), NULL, error, MAX_MJ_ERROR_LENGTH);
    if (mMujoco.model) {
        mMujoco.data = mj_makeData(mMujoco.model);
        mj_forward(mMujoco.model, mMujoco.data);
        mjv_makeScene(mMujoco.model, &mMujoco.scene, 1024);

        // mjr_makeContext(mMujoco.model, &mMujoco.context, 1);//50*(m_settings.font+1));

        // clear perturbation state
        mMujoco.perturb.active = 0;
        mMujoco.perturb.select = 0;
        mMujoco.perturb.skinselect = -1;

        mjv_defaultOption(&mMujoco.option);
        // align and scale view, update scene
        // alignscale();
        mjv_updateScene(mMujoco.model, mMujoco.data, &mMujoco.option, &mMujoco.perturb, &mMujoco.camera, mjCAT_ALL,
                        &mMujoco.scene);
        mjv_addGeoms(mMujoco.model, mMujoco.data, &mMujoco.option, &mMujoco.perturb, mjCAT_ALL, &mMujoco.scene);
        findMujocoObjectFromPrimPath();
    } else {
        std::cout << "Couldn't load " << fileName << std::endl;
        exit(0);
    }
    const int numPrims = (int)mUsdData.bodyPrims.size();
    printf("mMujoco.scene->ngeom (%d), numPrims (%d)\n", (int)mMujoco.scene.ngeom, (int)numPrims);

    pxr::UsdGeomXformCache xfCache;
    std::unique_ptr<pxr::SdfChangeBlock> changeBlock;
    IStageReaderWriter *iSip = carb::getCachedInterface<IStageReaderWriter>();
    StageReaderWriter stage = iSip->get(dataManager.mStageId);

    for (int i = 0; i < numPrims; ++i) {
        UsdGeomXformable xformable = UsdGeomXformable(mUsdData.bodyPrims[i]);
        // TODO: this is some hack to get the initial transforms right
        const GfMatrix4d currentPose = xfCache.GetLocalToWorldTransform(mUsdData.bodyPrims[i]);
        const GfTransform transform(currentPose);
        auto mat = GfMatrix4d();
        mat.SetIdentity();
        mat.SetTranslateOnly(transform.GetTranslation());
        mat.SetRotateOnly(transform.GetRotation().GetQuat());

        xformable.ClearXformOpOrder();
        const auto &transform_op = xformable.AddTransformOp(pxr::UsdGeomXformOp::PrecisionDouble);
        mXformOps.push_back(transform_op);
        const pxr::GfMatrix4d parentToWorldMat = xfCache.GetParentToWorldTransform(mUsdData.bodyPrims[i]);
        const pxr::GfMatrix4d worldToParentMat = parentToWorldMat.GetInverse();
        const pxr::GfMatrix4d local = mat * worldToParentMat;
        omni::usd::UsdUtils::setLocalTransformMatrix(mUsdData.bodyPrims[i], local, pxr::UsdTimeCode::Default(), false,
                                                     &changeBlock);
        // TODO : This is supposed to add the transform to the local transformation, and should be similar as above
        // call, but it isn't transform_op.Set(mat);

        const auto &scale_op = xformable.AddScaleOp(pxr::UsdGeomXformOp::PrecisionFloat);
        mScaleOps.push_back(scale_op);
        if (mUsdData.shapeMap.find(mUsdData.bodyPrims[i].GetPath()) != mUsdData.shapeMap.end()) {
            auto scale = mUsdData.shapeScales[mUsdData.bodyPrims[i].GetPath()];
            scale_op.Set(scale);
        }

        // Fabric related
        if (mUsdData.bodyPrims[i].HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
            dataManager.initializeFabricRigidBody(xfCache, mUsdData.bodyPrims[i], iSip, stage.getId());
        }
    }
}
// simulation was resumed
void CARB_ABI OmniMujocoUpdateNode::onResume(float currentTime) {
    std::cout << "OmniMuJoCoInterface::onResume" << std::endl;

    if (!mStageInitialized) {
        reset();
        omni::physics::schema::IUsdPhysics *usdPhysics =
            carb::getFramework()->acquireInterface<omni::physics::schema::IUsdPhysics>();
        if (usdPhysics) {
            pxr::UsdGeomXformCache xfCache;
            usdPhysics->registerPhysicsListener(this);
            pxr::UsdPrimRange range = mStage->Traverse(pxr::UsdTraverseInstanceProxies());
            omni::physics::schema::PrimIteratorRange primIteratorRange(range);
            usdPhysics->loadFromRange(mStage, xfCache, primIteratorRange);
            convertAndInitializeMujoco();
            if (mUsdData.scenePath != pxr::SdfPath()) {
                //"mSceneDesc.gravityDirection"
                std::cout << "mSceneDesc.gravityMagnitude=" << mUsdData.sceneDesc.gravityMagnitude << std::endl;
            }
            usdPhysics->unregisterPhysicsListener(this);
            mStageInitialized = true;
        } else {
            std::cout << "Error: couldn't find IUSdPhysics interface. Is omni.usdphysics enabled?" << std::endl;
        }
    }
}

void OmniMujocoUpdateNode::onDefaultUsdStageChanged(long stageId) {
    TfNotice::Revoke(mUsdNoticeListenerKey);
    mTimelineEventsSubscription = nullptr;
    mUpdateEventsSubscription = nullptr;
    mStage.Reset();

    if (stageId) {
        mStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
        mUsdNoticeListenerKey = TfNotice::Register(TfCreateWeakPtr(this), &OmniMujocoUpdateNode::onObjectsChanged);
    }
}

void OmniMujocoUpdateNode::onTimelineEvent(omni::timeline::TimelineEventType timelineEventType) {
    switch (timelineEventType) {
    case omni::timeline::TimelineEventType::ePlay: {
        // startAnimatingPrims();
    } break;
    case omni::timeline::TimelineEventType::eStop: {
        // stopAnimatingPrims();
    } break;
    default: {

    } break;
    }
}

// specific reset to this simulator
void OmniMujocoUpdateNode::reset() {
    mPaused = true;
    mStageInitialized = false;
    mUsdData.reset();
    mMujoco.reset();
    mXformOps.clear();
    mScaleOps.clear();
    mScales.clear();
    mScales.clear();
}
} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
