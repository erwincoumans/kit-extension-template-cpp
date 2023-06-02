// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include <carb/BindingsPythonUtils.h>

#include <omni/example/custom/physics/ICustomPhysicsInterface.h>

CARB_BINDINGS("omni.example.custom.physics.python")

DISABLE_PYBIND11_DYNAMIC_CAST(omni::example::custom::physics::ICustomPhysicsInterface)

namespace
{

// Define the pybind11 module using the same name specified in premake5.lua
PYBIND11_MODULE(_custom_physics_bindings, m)
{
    using namespace omni::example::custom::physics;

    m.doc() = "pybind11 omni.example.custom.physics bindings";

    carb::defineInterfaceClass<ICustomPhysicsInterface>(
        m, "ICustomPhysicsInterface", "acquire_custom_physics_interface", "release_custom_physics_interface")
        // .def("create_prims", &ICustomPhysicsInterface::createPrims)
        // .def("remove_prims", &ICustomPhysicsInterface::removePrims)
        // .def("print_stage_info", &ICustomPhysicsInterface::printStageInfo)
        // .def("start_timeline_animation", &ICustomPhysicsInterface::startTimelineAnimation)
        // .def("stop_timeline_animation", &ICustomPhysicsInterface::stopTimelineAnimation)
        .def("on_default_usd_stage_changed", &ICustomPhysicsInterface::onDefaultUsdStageChanged)
    /**/;
}
}
