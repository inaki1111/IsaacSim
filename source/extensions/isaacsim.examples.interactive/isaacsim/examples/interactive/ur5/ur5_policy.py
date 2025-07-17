from typing import Optional
import os
import numpy as np

from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.robot.policy.examples.controllers import PolicyController


class UR5GraspPolicy(PolicyController):
    def __init__(
        self,
        prim_path: str,
        obj,
        root_path: Optional[str] = None,
        name: str = "ur5",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        if usd_path is None:
            usd_path = os.path.join(os.path.dirname(__file__), "ur5.usdz")
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        policy_dir = os.path.dirname(__file__)
        self.load_policy(
            os.path.join(policy_dir, "policy.pt"),
            os.path.join(policy_dir, "env.yaml"),
        )

        self.object           = obj
        self.ee_prim          = None
        self.obj_prim         = None
        self._previous_action = np.zeros(8, dtype=np.float32)
        self._policy_counter  = 0

        # offset of the end effector
        self.pos_offset_ee   = np.zeros(3, dtype=np.float32)
        self.orn_offset_ee   = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def initialize(self, physics_sim_view=None) -> None:
        super().initialize(
            physics_sim_view=physics_sim_view,
            control_mode="force",
            set_articulation_props=True,
        )
        if physics_sim_view is not None:
            self.object.initialize(physics_sim_view=physics_sim_view)
            self.ee_prim  = get_prim_at_path(f"{self.robot.prim_path}/wrist_3_link")
            self.obj_prim = get_prim_at_path(self.object.prim_path)
        self.robot.set_solver_position_iteration_count(32)
        self.robot.set_solver_velocity_iteration_count(4)
        self.robot.set_stabilization_threshold(0)
        self.robot.set_sleep_threshold(0)

    def _compute_observation(self) -> np.ndarray:
        jp = self.robot.get_joint_positions()
        jv = self.robot.get_joint_velocities()
        obs = np.zeros(42, dtype=np.float32)

        # Joint positions & velocities
        obs[0:12]  = jp[:12] - self.default_pos[:12]
        obs[12:24] = jv[:12] - self.default_vel[:12]

        # Position of the object and target hardcoded for testing
        # Direct assignment without querying environment
        #pos_obj = (0.5, 0.0, 0.0)
        pos_obj, quat_obj = self.object.get_world_pose()
        #print(f"Object position: {pos_obj}, orientation: {quat_obj}")

        obs[24:27] = np.array(pos_obj, dtype=np.float32)

        goal_pos = (0.5, 0.5, 0.5)

        goal_quat = (1.0, 0.0, 0.0, 0.0)
        obs[27:30] = np.array(goal_pos, dtype=np.float32)
        obs[30:34] = np.array(goal_quat, dtype=np.float32)

        # Last action taken
        obs[34:42] = self._previous_action

        #print("Observation:", obs)
        return obs

    def forward(self, dt: float) -> None:
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            a   = self._compute_action(obs)
            self._previous_action = a.copy()

            # Pose and gripper outputs
            pose_raw    = a[:7]
            gripper_raw = float(a[7])

            pos_des     = pose_raw[:3] + self.pos_offset_ee
            quat        = pose_raw[3:7]

            self.desired_pose   = np.concatenate([pos_des, quat]).astype(np.float32)
            self.gripper_action = 1.0 if gripper_raw > 0.0 else 0.0

        self._policy_counter += 1

    def _get_target_command(self) -> np.ndarray:
        raise NotImplementedError("error")
