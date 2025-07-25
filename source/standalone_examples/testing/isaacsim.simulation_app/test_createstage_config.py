# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaacsim import SimulationApp

# Test app startup without creating new stage
kit = SimulationApp({"create_new_stage": False})

import isaacsim.core.utils.stage as stage_utils
import omni

for i in range(100):
    kit.update()

omni.kit.app.get_app().print_and_log("Config: No empty stage was created")

stage_utils.create_new_stage()

for i in range(100):
    kit.update()

kit.close()  # Cleanup application
