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

#include <carb/Interface.h>
#include <carb/Interface.h>

namespace omni {
namespace example {
namespace custom {
namespace physics {

class UsdData;
/**
 * Interface used to interact with the example C++ plugin from Python.
 */
class ICustomPhysicsInterface {
  public:
    CARB_PLUGIN_INTERFACE("omni::example::custom::physics::ICustomPhysicsInterface", 1, 0);

    /**
     * Called when the default USD stage (ie. the one open in the main viewport) changes.
     * Necessary for now until the omni.usd C++ API becomes ready for public consumption.
     *
     * @param stageId The id of the new default USD stage.
     */
    virtual void onDefaultUsdStageChanged(long stageId) = 0;

    /// Update the physics simulation. All scenes, except the ones marked as 'Disabled', are updated and stepped.
    ///
    /// \param[in] elapsedStep Simulation time.
    /// \param[in] currentTime Current time, might be used for time sampled transformations to apply.
    void(CARB_ABI* updateSimulation)(float elapsedStep, float currentTime);

    // /// Update all the transformations. All scenes, except the ones marked as 'Disabled', are updated and stepped.
    // /// Note: the 'outputVelocitiesLocalSpace' parameter has no effect and will be removed in a future version.
    // ///
    // /// \param[in] updateToFastCache            Update transforms to fast cache. DEPRECATED, fast cache is not used anymore
    // /// \param[in] updateToUsd                  Update transforms to USD.
    // /// \param[in] updateVelocitiesToUsd        Update velocities to USD.
    // void(CARB_ABI* updateTransformations)(bool updateToFastCache, bool updateToUsd, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace);

    /// Start simulation, store initial USD data
    void(CARB_ABI* startSimulation)();

    /// Reset simulation, set back USD start data
    void(CARB_ABI* resetSimulation)();

    /// Get USD physics data
    UsdData* (CARB_ABI* getPhysicsUsdData)();

};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
