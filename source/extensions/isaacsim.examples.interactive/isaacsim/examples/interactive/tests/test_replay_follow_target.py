# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import os

import omni.kit
import omni.kit.test

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import create_new_stage_async, is_stage_loading, update_stage_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.examples.interactive.replay_follow_target import ReplayFollowTarget


class TestReplayFollowTargetExampleExtension(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        await update_stage_async()
        self._sample = ReplayFollowTarget()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await update_stage_async()
        while is_stage_loading():
            await update_stage_async()
        return

    # After running each test
    async def tearDown(self):
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while is_stage_loading():
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await self._sample.clear_async()
        await update_stage_async()
        self._sample = None
        pass

    async def test_reset(self):
        await self._sample.reset_async()
        await update_stage_async()
        await update_stage_async()
        await self._sample.reset_async()
        await update_stage_async()
        await update_stage_async()
        pass

    async def test_replay_trajectory(self):
        await self._sample.reset_async()
        await update_stage_async()

        await self._sample._on_replay_trajectory_event_async(
            os.path.abspath(
                os.path.join(
                    get_extension_path_from_name("isaacsim.examples.interactive"), "data/example_data_file.json"
                )
            )
        )
        await update_stage_async()
        # run for 2500 frames and print time
        for i in range(500):
            await update_stage_async()
        pass

    async def test_replay_scene(self):
        await self._sample.reset_async()
        await update_stage_async()
        await self._sample._on_replay_scene_event_async(
            os.path.abspath(
                os.path.join(
                    get_extension_path_from_name("isaacsim.examples.interactive"), "data/example_data_file.json"
                )
            )
        )
        await update_stage_async()
        # run for 2500 frames and print time
        for i in range(500):
            await update_stage_async()
        pass
