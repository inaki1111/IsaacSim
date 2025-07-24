# ur5_example.py

import os
import numpy as np
import omni

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.examples.interactive.base_sample import BaseSample
from .ur5controller import UR5Controller
from .ur5_action_graph import create_ur5_action_graph_for_joint_state

class UR5GraspExample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # Set units and simulation timesteps
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 400.0
        self._world_settings["rendering_dt"] = 1.0 / 60.0

    def setup_scene(self) -> None:
        # 1) Add a default ground plane
        self.get_world().scene.add_default_ground_plane(
            z_position=-0.63,
            name="ground_plane",
            prim_path="/World/groundPlane",
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        )

        # 2) Create the ROS2 <-> OmniGraph bridge for JointState messages
        create_ur5_action_graph_for_joint_state(
            pub_topic="/joint_states_command",
            sub_topic="joint_states"
        )

        dir_path = os.path.dirname(__file__)

        # 3) Load a table model (optional)
        table_prim = "/World/Table"
        table_usd = os.path.join(dir_path, "table.usd")
        add_reference_to_stage(table_usd, table_prim)
        self.table = SingleArticulation(
            prim_path=table_prim,
            name="Table",
            position=np.array([0.5, 0.0, -0.63]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        # 4) Load the UR5 robot from USD
        ur5_prim = "/World/ur5"
        ur5_usd = os.path.join(dir_path, "ur5.usd")
        add_reference_to_stage(ur5_usd, ur5_prim)
        self.ur5 = SingleArticulation(
            prim_path=ur5_prim,
            name="ur5",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        # 5) Initialize the RMP-Flow controller with joint names
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.controller = UR5Controller(self.ur5, joint_names)

        # 6) Subscribe to the STOP event to reset physics readiness
        timeline = omni.timeline.get_timeline_interface()
        self._timer_sub = (
            timeline.get_timeline_event_stream()
                    .create_subscription_to_pop_by_type(
                        int(omni.timeline.TimelineEventType.STOP),
                        self._on_timeline_stop,
                    )
        )

    async def setup_post_load(self) -> None:
        # Register physics callback after scene load
        self._physics_ready = False
        self.get_world().add_physics_callback(
            "physics_step", callback_fn=self._on_physics
        )
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        # Reset physics readiness after a world reset
        self._physics_ready = False
        await self.get_world().play_async()

    def _on_physics(self, step_size: float) -> None:
        if not self._physics_ready:
            # First physics step: set an initial joint state (all zeros)
            initial_positions = np.zeros(6, dtype=np.float32)
            self.ur5.set_joints_default_state(initial_positions)
            self._physics_ready = True
            return

        # On each subsequent physics step, drive the end effector to (0.5, 0.5, 0.5)
        # and publish the resulting joint states
        self.controller.apply_action(None)

    def _on_timeline_stop(self, event) -> None:
        # Reset the flag so that next run re-initializes joint states
        self._physics_ready = False

    def world_cleanup(self) -> None:
        # Remove physics callback and subscription when done
        world = self.get_world()
        if world.physics_callback_exists("physics_step"):
            world.remove_physics_callback("physics_step")
        self._timer_sub = None
