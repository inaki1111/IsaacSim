# ur5_rmpflow_example.py

import os
import numpy as np
import omni
from pxr import Sdf, UsdLux

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
from isaacsim.core.api.world import World
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
import omni.graph.core as og

from .ur5_action_graph import create_ur5_action_graph_for_joint_state

class UR5RmpflowExample(BaseSample):
    def __init__(self):
        super().__init__()
        # Configure world settings: units and time steps
        self._world_settings = {
            "stage_units_in_meters": 1.0,
            "physics_dt": 1.0 / 400.0,
            "rendering_dt": 1.0 / 60.0,
        }
        # Define joint names for the UR5
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self._physics_ready = False

    def setup_scene(self) -> None:
        # Stage is already created by BaseSample; add a directional light
        UsdLux.SphereLight.Define(
            get_current_stage(), Sdf.Path("/World/SphereLight")
        ).CreateIntensityAttr(100000)

        # Get the world from BaseSample
        world = self.get_world()

        # Add a default ground plane
        world.scene.add_default_ground_plane(
            z_position=0.0
            name="ground_plane",
            prim_path="/World/groundPlane",
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        )

        # Load the UR5 robot from USD and add to the scene
        base_dir = os.path.dirname(__file__)
        ur5_usd = os.path.join(base_dir, "ur5.usdz")
        add_reference_to_stage(ur5_usd, "/World/ur5")
        self.robot = SingleArticulation(
            prim_path="/World/ur5",
            name="ur5",
            position=np.array([-0.16810509302823873, 0.5859542808301269, 0.26483031099633514]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        world.scene.add(self.robot)

        # Create a red cuboid target in the scene
        self.cuboid = VisualCuboid(
            "/World/cuboid",
            position=np.array([-0.5, -0.5, 0.5]),
            size=0.05,
            color=np.array([255, 0, 0]),
        )
        world.scene.add(self.cuboid)

        # Setup ROS2 <-> OmniGraph bridge for joint_state commands
        create_ur5_action_graph_for_joint_state(
            pub_topic="/joint_states_command", sub_topic="joint_states"
        )

        # Initialize RMP-Flow controller and motion policy
        self.rmpflow = RmpFlow(
            # Path to URDF file for kinematic description
            urdf_path=os.path.join(base_dir, "ur5.urdf"),
            # Path to robot description yaml (SRDF or Lula-generated yaml)
            robot_description_path=os.path.join(base_dir, "ur5_lula.yaml"),
            # RMP-Flow configuration file
            rmpflow_config_path=os.path.join(base_dir, "rmpflow.yaml"),
            end_effector_frame_name="wrist_3_link",
            maximum_substep_size=self._world_settings["physics_dt"],
        )
        self.motion_policy = ArticulationMotionPolicy(
            self.robot, self.rmpflow, self._world_settings["rendering_dt"]
        )


        # Subscribe to physics callback for control loop
        world.add_physics_callback("physics_step", self._on_physics)

    async def setup_post_load(self) -> None:
        # Start the simulation after scene load
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        # Reset physics readiness on world reset
        self._physics_ready = False
        await self.get_world().play_async()

    def _on_physics(self, step_size: float) -> None:
        # Initialize joint state on the first physics step
        if not self._physics_ready:
            zero_q = np.zeros(len(self.joint_names), dtype=np.float32)
            self.robot.set_joints_default_state(zero_q)
            self._physics_ready = True
            return

        # Read the cuboid world position
        cube_pos, _ = self.cuboid.get_world_pose()

        # Set the RMP-Flow end-effector target to the cuboid position
        self.rmpflow.set_end_effector_target(target_position=cube_pos)

        # Compute the next articulation action
        art_action = self.motion_policy.get_next_articulation_action()
        if art_action.joint_positions is None:
            return

        # Apply high gains for precise tracking
        n = len(self.joint_names)
        kp = 1e15 * np.ones(n)
        kd = 1e14 * np.ones(n)
        self.robot.get_articulation_controller().set_gains(kp, kd)

        # Apply the computed action to the robot
        self.robot.apply_action(art_action)

        # Publish joint commands via OmniGraph to ROS2
        jp = art_action.joint_positions.tolist()
        jv = art_action.joint_velocities.tolist()
        og.Controller.set(
            "/ActionGraph/PublishJointState.inputs:message",
            {
                "name": "joint_states_command",
                "position": jp,
                "velocity": jv,
                "effort": [],
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                "joint_names": self.joint_names,
            },
        )

    def world_cleanup(self) -> None:
        # Remove physics callback when cleaning up
        world = self.get_world()
        if world.physics_callback_exists("physics_step"):
            world.remove_physics_callback("physics_step")
