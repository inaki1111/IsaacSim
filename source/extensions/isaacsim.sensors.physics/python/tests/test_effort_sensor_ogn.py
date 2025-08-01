# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import asyncio
import sys

import carb
import isaacsim.core.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage_async
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdPhysics


class TestEffortSensorOgn(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await create_new_stage_async()
        await self.setUp_environment()
        await self.setup_ogn()

        physics_rate = 60
        self.my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / physics_rate, rendering_dt=1.0 / physics_rate)
        await self.my_world.initialize_simulation_context_async()

    async def tearDown(self):
        if self.my_world:
            self.my_world.stop()
            self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def setUp_environment(self):

        assets_root_path = get_assets_root_path()

        asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/Articulation")
        arm_joint = "/Articulation/Arm/RevoluteJoint"
        arm_prim = get_prim_at_path(arm_joint)
        joint = UsdPhysics.RevoluteJoint(arm_prim)
        joint.CreateAxisAttr("Y")

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadEffortNode", "isaacsim.sensors.physics.IsaacReadEffortSensor"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadEffortNode.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying force value and sensor time are non-zero in valid case
    async def test_valid_effort_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadEffortNode.inputs:prim"),
            [usdrt.Sdf.Path("/Articulation/Arm/RevoluteJoint")],
        )
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(0.5)
        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertNotEqual(effort_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    # verifying that force value and sensor time equal zero in invalid case

    async def test_invalid_effort_sensor_ogn(self):

        self.my_world.play()
        await simulate_async(0.5)

        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertEqual(effort_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:sensorTime").get()
        self.assertEqual(sensor_time, 0.0)
