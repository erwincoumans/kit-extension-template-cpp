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

#include "DataManager.h"
#include <mujoco/mujoco.h>
#include <omni/usd/UsdUtils.h>
#include <physxSchema/physxSceneAPI.h>

namespace omni {
namespace example {
namespace custom {
namespace physics {

using namespace PXR_NS;
using namespace carb;

DataManager::DataManager() {
    mStageId.id = 0;
    omni::fabric::Token::iToken = getFramework()->tryAcquireInterface<omni::fabric::IToken>();
    omni::fabric::Path::iPath = getFramework()->tryAcquireInterface<omni::fabric::IPath>();
    mWorldPosToken = omni::fabric::Token::iToken->getHandle(gWorldPositionTokenString.c_str());
    mWorldOrientToken = omni::fabric::Token::iToken->getHandle(gWorldOrientationTokenString.c_str());
    mWorldScaleToken = omni::fabric::Token::iToken->getHandle(gWorldScaleTokenString.c_str());
    mMujocoBodyToken = omni::fabric::Token::iToken->getHandle(gMujocoBodyString.c_str());
    mFloat3Type = omni::fabric::Type(omni::fabric::BaseDataType::eFloat, 3, 0, omni::fabric::AttributeRole::eNone);
    mDouble3Type = omni::fabric::Type(omni::fabric::BaseDataType::eDouble, 3, 0, omni::fabric::AttributeRole::eNone);
    mQuatType = omni::fabric::Type(omni::fabric::BaseDataType::eFloat, 4, 0, omni::fabric::AttributeRole::eQuaternion);
}

DataManager::~DataManager() {
    // pause();
    mStageId.id = 0;
}

void DataManager::initializeFabricRigidBody(pxr::UsdGeomXformCache &xfCache, const pxr::UsdPrim &prim,
                                            IStageReaderWriter *iStageReaderWriter,
                                            StageReaderWriterId stageInProgress) {

    IStageReaderWriter *iSip = carb::getCachedInterface<IStageReaderWriter>();
    StageReaderWriter stage = iSip->get(mStageId);

    pxr::UsdPhysicsRigidBodyAPI rboAPI(prim);
    if (rboAPI) {
        bool kinematic = false;
        rboAPI.GetKinematicEnabledAttr().Get(&kinematic);
        if (!kinematic) {
            const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(prim);
            const GfTransform transform(worldPose);
            const omni::fabric::PathC primPath = omni::fabric::asInt(prim.GetPrimPath());
            iStageReaderWriter->prefetchPrim(mStageId, primPath);
            iStageReaderWriter->createAttribute(stageInProgress, primPath, mWorldPosToken,
                                                (omni::fabric::TypeC)mDouble3Type);
            iStageReaderWriter->createAttribute(stageInProgress, primPath, mWorldOrientToken,
                                                (omni::fabric::TypeC)mQuatType);
            Float3 &scaleData =
                *(Float3 *)(iStageReaderWriter->getOrCreateAttributeWr(stageInProgress, primPath, mWorldScaleToken,
                                                                       (omni::fabric::TypeC)mFloat3Type))
                     .ptr;
            Double3 &posData =
                *(Double3 *)(iStageReaderWriter->getAttributeWr(stageInProgress, primPath, mWorldPosToken)).ptr;
            Float4 &orientData =
                *(Float4 *)(iStageReaderWriter->getAttributeWr(stageInProgress, primPath, mWorldOrientToken)).ptr;

            const GfVec3d scale = transform.GetScale();
            scaleData.x = float(scale[0]);
            scaleData.y = float(scale[1]);
            scaleData.z = float(scale[2]);

            const GfVec3d wPos = transform.GetTranslation();
            posData.x = wPos[0];
            posData.y = wPos[1];
            posData.z = wPos[2];
            // std::cout << " initializing " << prim.GetPrimPath().GetText() << wPos << std::endl;

            const GfQuatd wRot = transform.GetRotation().GetQuat();
            orientData.x = float(wRot.GetImaginary()[0]);
            orientData.y = float(wRot.GetImaginary()[1]);
            orientData.z = float(wRot.GetImaginary()[2]);
            orientData.w = float(wRot.GetReal());

            Float3 pos = {float(wPos[0]), float(wPos[1] + 1), float(wPos[2] + 1)};
            Float4 orient = {float(wRot.GetImaginary()[0]), float(wRot.GetImaginary()[1]),
                             float(wRot.GetImaginary()[2]), float(wRot.GetReal())};
            mInitialTransformation[primPath.path] = std::make_pair(pos, orient);
            StageReaderWriter stage(stageInProgress);
            stage.createTag(primPath, mMujocoBodyToken);
        }
    }
}

void DataManager::updateFabric(MujocoData &mMujoco, UsdData &mUsdData, pxr::UsdStageRefPtr mStage,
                               bool mStageInitialized, float currentTime, float elapsedSecs) {
    if (!mStageInitialized)
        return;

    pxr::PhysxSchemaPhysxSceneAPI physxSceneAPI =
        pxr::PhysxSchemaPhysxSceneAPI::Apply(mStage->GetPrimAtPath(mUsdData.scenePath));
    uint32_t timeStepsPerSeconds;
    physxSceneAPI.GetTimeStepsPerSecondAttr().Get(&timeStepsPerSeconds);
    double fixedTimeStep = 1.0 / timeStepsPerSeconds;
    double actualFreq = 1.0 / fixedTimeStep;
    double totalElapsedTime = elapsedSecs + mRemainderTime;
    uint32_t numSteps = (uint32_t)(totalElapsedTime / fixedTimeStep);
    carb::settings::ISettings *settings = carb::getCachedInterface<carb::settings::ISettings>();
    const char kMinFrameRate[] = PERSISTENT_SETTINGS_PREFIX "/simulation/minFrameRate";
    const int minFrameRate = settings->getAsInt(kMinFrameRate);
    if (minFrameRate) {
        const uint32_t minSteps = timeStepsPerSeconds / minFrameRate;
        const uint32_t maxSteps = minSteps > 0 ? minSteps : 1;
        numSteps = numSteps > maxSteps ? maxSteps : numSteps;
        mRemainderTime = fminf(totalElapsedTime - fixedTimeStep * numSteps, maxSteps * fixedTimeStep);
    } else {
        mRemainderTime = totalElapsedTime - fixedTimeStep * numSteps;
    }

    mMujoco.model->opt.timestep = fixedTimeStep;
    for (uint32_t i = 0; i < numSteps; i++) {
        mj_step(mMujoco.model, mMujoco.data);
        mjv_updateScene(mMujoco.model, mMujoco.data, &mMujoco.option, &mMujoco.perturb, &mMujoco.camera, mjCAT_ALL,
                        &mMujoco.scene);
    }

    const Type typePrimName(BaseDataType::eTag, 1, 0, AttributeRole::ePrimTypeName);
    const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
    const Type typeAppliedType(BaseDataType::eTag, 1, 0, AttributeRole::eNone);
    const Type typeDouble3(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
    const Type typeQuat(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);
    const Token tokenRigidBody("PhysicsRigidBodyAPI");
    IStageReaderWriter *iSip = carb::getCachedInterface<IStageReaderWriter>();
    StageReaderWriter stage = iSip->get(mStageId);

    const omni::fabric::set<AttrNameAndType_v2> requiredAll = {AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody),
                                                               AttrNameAndType_v2(typeAppliedType, mMujocoBodyToken)};
    const omni::fabric::set<AttrNameAndType_v2> requiredAny = {AttrNameAndType_v2(typeDouble3, mWorldPosToken),
                                                               AttrNameAndType_v2(typeQuat, mWorldOrientToken)};

    PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(mStage);

    for (size_t i = 0; i != bucketCount; i++) {
        gsl::span<carb::Double3> positions = stage.getAttributeArray<carb::Double3>(primBuckets, i, mWorldPosToken);
        gsl::span<carb::Float4> orientations = stage.getAttributeArray<carb::Float4>(primBuckets, i, mWorldOrientToken);

        // TODO: use vectorized API
        gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
        size_t j = 0;
        for (const omni::fabric::Path &path : paths) {
            const pxr::SdfPath usdPath = omni::fabric::intToPath(path);
            TransformationCache::const_iterator fit = mInitialTransformation.find(PathC(path).path);
            if (fit != mInitialTransformation.end()) {

                int geomIndex = mMujoco.pathsToIds[usdPath];
                if (geomIndex > -1) {
                    auto mat = GfMatrix4d();
                    mat.SetIdentity();
                    GfVec3d pos = 1.0 / metersPerUnit *
                                  GfVec3d(mMujoco.scene.geoms[geomIndex].pos[0], mMujoco.scene.geoms[geomIndex].pos[1],
                                          mMujoco.scene.geoms[geomIndex].pos[2]);
                    double v[9] = {
                        (double)mMujoco.scene.geoms[geomIndex].mat[0], (double)mMujoco.scene.geoms[geomIndex].mat[1],
                        (double)mMujoco.scene.geoms[geomIndex].mat[2], (double)mMujoco.scene.geoms[geomIndex].mat[3],
                        (double)mMujoco.scene.geoms[geomIndex].mat[4], (double)mMujoco.scene.geoms[geomIndex].mat[5],
                        (double)mMujoco.scene.geoms[geomIndex].mat[6], (double)mMujoco.scene.geoms[geomIndex].mat[7],
                        (double)mMujoco.scene.geoms[geomIndex].mat[8]};
                    auto rot = GfMatrix3d(v[0], v[3], v[6], v[1], v[4], v[7], v[2], v[5], v[8]);
                    // mat.SetScale(scale);
                    mat.SetTranslateOnly(pos);
                    mat.SetRotateOnly(rot);
                    pxr::GfTransform trans(mat);
                    pxr::GfQuatd q = trans.GetRotation().GetQuat();
                    positions[j] = {pos[0], pos[1], pos[2]};
                    orientations[j] = {(float)q.GetImaginary()[0], (float)q.GetImaginary()[1],
                                       (float)q.GetImaginary()[2], (float)q.GetReal()};
                    j++;
                }
            }
        }
    }
}

// this gets called in every update cycle of Kit (typically at each render frame)
void DataManager::updateUsd(MujocoData &mMujoco, UsdData &mUsdData, pxr::UsdStageRefPtr mStage, bool mStageInitialized,
                            float currentTime, float elapsedSecs) {
    if (!mStageInitialized)
        return;
    mj_step(mMujoco.model, mMujoco.data);
    mjv_updateScene(mMujoco.model, mMujoco.data, &mMujoco.option, &mMujoco.perturb, &mMujoco.camera, mjCAT_ALL,
                    &mMujoco.scene);

    const int numPrims = (int)mUsdData.bodyPrims.size();
    pxr::UsdGeomXformCache xfCache;
    std::unique_ptr<pxr::SdfChangeBlock> changeBlock;
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(mStage);
    for (int geomIndex = 0; geomIndex < mMujoco.scene.ngeom; ++geomIndex) {
        auto ppath = mMujoco.idsToPaths[geomIndex];
        if (mMujoco.idsToPaths[geomIndex] != pxr::SdfPath() &&
            mUsdData.rigidBodyMap.find(ppath) != mUsdData.rigidBodyMap.end()) {
            auto prim = mStage->GetPrimAtPath(ppath);
            UsdGeomXformable xformable = UsdGeomXformable(prim);
            auto mat = GfMatrix4d();
            mat.SetIdentity();
            // int geomIndex = i; // skip ground plane for now
            GfVec3d pos = 1.0 / metersPerUnit *
                          GfVec3d(mMujoco.scene.geoms[geomIndex].pos[0], mMujoco.scene.geoms[geomIndex].pos[1],
                                  mMujoco.scene.geoms[geomIndex].pos[2]);

            double v[9] = {(double)mMujoco.scene.geoms[geomIndex].mat[0], (double)mMujoco.scene.geoms[geomIndex].mat[1],
                           (double)mMujoco.scene.geoms[geomIndex].mat[2], (double)mMujoco.scene.geoms[geomIndex].mat[3],
                           (double)mMujoco.scene.geoms[geomIndex].mat[4], (double)mMujoco.scene.geoms[geomIndex].mat[5],
                           (double)mMujoco.scene.geoms[geomIndex].mat[6], (double)mMujoco.scene.geoms[geomIndex].mat[7],
                           (double)mMujoco.scene.geoms[geomIndex].mat[8]};
            auto rot = GfMatrix3d(v[0], v[3], v[6], v[1], v[4], v[7], v[2], v[5], v[8]);

            mat.SetTranslateOnly(pos);
            mat.SetRotateOnly(rot);
            if (mUsdData.shapeMap.find(prim.GetPath()) != mUsdData.shapeMap.end()) {
                auto scale = mUsdData.shapeScales[prim.GetPath()];
                auto scaleMat = GfMatrix4d().SetScale(scale);
                mat = scaleMat * mat;
                // mScaleOps[geomIndex].Set(scale);
            }
            // mXformOps[geomIndex].Set(mat);

            const pxr::GfMatrix4d parentToWorldMat = xfCache.GetParentToWorldTransform(prim);
            const pxr::GfMatrix4d worldToParentMat = parentToWorldMat.GetInverse();
            const pxr::GfMatrix4d local = mat * worldToParentMat;
            omni::usd::UsdUtils::setLocalTransformMatrix(prim, local, pxr::UsdTimeCode::Default(), false, &changeBlock);
        }
    }
}
} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni