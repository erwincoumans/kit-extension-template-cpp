## Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
##
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.
##
import omni.kit.test
import omni.kit.app

import omni.usd
import omni.example.custom.physics


class TestUsdExample(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        # Cache the example usd interface.
        self.custom_physics_interface = omni.example.custom.physics.get_custom_physics_interface()

        # Open a new USD stage.
        omni.usd.get_context().new_stage()
        self.usd_stage = omni.usd.get_context().get_stage()

    async def tearDown(self):
        # Close the USD stage.
        await omni.usd.get_context().close_stage_async()
        self.usd_stage = None

        # Clear the example usd interface.
        self.custom_physics_interface = None

    async def test_create_prims(self):
        # self.custom_physics_interface.create_prims()
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_0"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_1"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_2"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_3"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_4"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_5"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_6"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_7"))
        # self.assertTrue(self.usd_stage.GetPrimAtPath("/World/example_prim_8"))
        # self.assertFalse(self.usd_stage.GetPrimAtPath("/World/a_random_prim"))
