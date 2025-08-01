# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton, Button
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.storage.native import get_assets_root_path
from omni.usd import StageEventType
from pxr import Sdf, UsdLux
from isaacsim.core.prims import SingleArticulation, RigidPrim, XFormPrim
#from omni.isaac.core.Articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy

class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()
        self.rmpflow = None
        self.articulation_rmpflow = None
        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
            # For complete robustness, the user should resolve those edge cases here
            # In general, for extensions based off this template, there is no value to having the user click the play/stop
            # button instead of using the Load/Reset/Run buttons provided.
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        self.on_every_frame()

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton(
                    "Load Button", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario
                )
                self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario")

        with run_scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

        goto_target_frame = CollapsableFrame("Go To Target")
        with goto_target_frame:
            use_rmpflow_btn = Button("Use RMPFlow", "RUN", on_click_fn=self.on_pressed_rmpflow_fn)
    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def on_pressed_rmpflow_fn(self):
        if self._articulation is None:
            return
        

                # Create a cuboid
        self.cuboid = FixedCuboid("/cuboid", position=np.array([0.5, 0.0, 0.0]), size=0.1, color=np.array([255, 0, 0]))

        self.rmpflow = RmpFlow(
            robot_description_path="/home/inaki/isaacsim/ur5/ur5_extension/UR5_Load_scene_python/ur5_lula.yaml",
            urdf_path="/home/inaki/isaacsim/ur5/ur5_extension/UR5_Load_scene_python/ur5.urdf",
            rmpflow_config_path="/home/inaki/isaacsim/ur5/ur5_extension/UR5_Load_scene_python/rmpflow.yaml",
            end_effector_frame_name="wrist_3_link",
            maximum_substep_size=0.0034
        )

        self.articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self.rmpflow, 1/60)
        print("RMPFlow initialized")

    def on_every_frame(self):
        if self.rmpflow is None:
            return
        
        self.rmpflow.set_end_effector_target(
            target_position=self.cuboid.get_world_pose()[0],
            target_position=self.cuboid.get_world_pose()[1]
        )

        actions = self.articulation_rmpflow.get_next_articulation_action()
        self._articulation.get_articulation_controller(1e15*np.ones(6),1e14*np.ones(6))
        self._articulation.apply_action(actions)





    def _on_init(self):
        self._articulation = None
        self._cuboid = None

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath())).set_world_poses(np.array([[6.5, 0, 12]]))

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.

        In this example, a new stage is loaded explicitly, and all assets are reloaded.
        If the user is relying on hot-reloading and does not want to reload assets every time,
        they may perform a check here to see if their desired assets are already on the stage,
        and avoid loading anything if they are.  In this case, the user would still need to add
        their assets to the World (which has low overhead).  See commented code section in this function.
        """
        # Load the UR10e
        robot_prim_path = "/ur5"
        path_to_robot_usd = "/home/inaki/isaacsim/ur5/ur5_extension/UR5_Load_scene_python/ur5_standalone.usd"

        # Do not reload assets when hot reloading.  This should only be done while extension is under development.
        # if not is_prim_path_valid(robot_prim_path):
        #     create_new_stage()
        #     add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        # else:
        #     print("Robot already on Stage")



        create_new_stage()
        self._add_light_to_stage()
        add_reference_to_stage(path_to_robot_usd, robot_prim_path)


        self._articulation = SingleArticulation(robot_prim_path)

        # Add user-loaded objects to the World
        world = World.instance()
        world.scene.add(self._articulation)
        world.scene.add(self._cuboid)

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.

        In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
        provided prim in a circle around the robot.
        """
        self._reset_scenario()

        # UI management
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _reset_scenario(self):
        self._scenario.teardown_scenario()
        self._scenario.setup_scenario(self._articulation, self._cuboid)

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        self._reset_scenario()

        # UI management
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        Args:
            step (float): The dt of the current physics step
        """
        self._scenario.update_scenario(step)

    def _on_run_scenario_a_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_a_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "RUN".

        This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
        the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
        through the left-hand UI toolbar.
        """
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
