## Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
##
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.
##
import omni.ext
import omni.usd
from .._custom_physics_bindings import *

# Global public interface object.
_custom_physics_interface = None

# Public API.
def get_custom_physics_interface() -> ICustomPhysicsInterface:
    return _custom_physics_interface


# Use the extension entry points to acquire and release the interface,
# and to subscribe to usd stage events.
class CustomPhysicsExtension(omni.ext.IExt):
    def on_startup(self):
        # Acquire the example USD interface.
        global _custom_physics_interface
        _custom_physics_interface = acquire_custom_physics_interface()

        # Inform the C++ plugin if a USD stage is already open.
        usd_context = omni.usd.get_context()
        if usd_context.get_stage_state() == omni.usd.StageState.OPENED:
            _custom_physics_interface.on_default_usd_stage_changed(usd_context.get_stage_id())

        # Subscribe to omni.usd stage events so we can inform the C++ plugin when a new stage opens.
        self._stage_event_sub = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="omni.example.custom.physics"
        )

        # Print some info about the stage from C++.
        # _custom_physics_interface.print_stage_info()

        # Create some example prims from C++.
        # _custom_physics_interface.create_prims()

        # Print some info about the stage from C++.
        # _custom_physics_interface.print_stage_info()

        # Animate the example prims from C++.
        # _custom_physics_interface.start_timeline_animation()

    def on_shutdown(self):
        global _custom_physics_interface

        # # Stop animating the example prims from C++.
        # _custom_physics_interface.stop_timeline_animation()

        # # Remove the example prims from C++.
        # _custom_physics_interface.remove_prims()

        # Unsubscribe from omni.usd stage events.
        self._stage_event_sub = None

        # Release the example USD interface.
        release_custom_physics_interface(_custom_physics_interface)
        _custom_physics_interface = None

    def _on_stage_event(self, event):
        # if event.type == int(omni.usd.StageEventType.OPENED):
        #     _custom_physics_interface.on_default_usd_stage_changed(omni.usd.get_context().get_stage_id())
        # elif event.type == int(omni.usd.StageEventType.CLOSED):
        #     _custom_physics_interface.on_default_usd_stage_changed(0)
        return