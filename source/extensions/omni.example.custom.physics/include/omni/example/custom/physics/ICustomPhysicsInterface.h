// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#pragma once

#include <carb/Interface.h>

namespace omni {
namespace example {
namespace custom {
namespace physics {

/**
 * Interface used to interact with the example C++ USD plugin from Python.
 */
class ICustomPhysicsInterface {
  public:
    /// @private
    CARB_PLUGIN_INTERFACE("omni::example::custom::physics::ICustomPhysicsInterface", 1, 0);

    /**
     * Called when the default USD stage (ie. the one open in the main viewport) changes.
     * Necessary for now until the omni.usd C++ API becomes ready for public consumption.
     *
     * @param stageId The id of the new default USD stage.
     */
    virtual void onDefaultUsdStageChanged(long stageId) = 0;
};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
