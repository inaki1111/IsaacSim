# ur5_grasp_policy.py
from typing import Optional
import os
import numpy as np
import omni.graph.core as og

from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.robot.policy.examples.controllers import PolicyController
from .ur5_action_graph import GRAPH_PATH, NODE_SUB

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

        #load the policy
        dir_path = os.path.dirname(__file__)
        self.load_policy(
            os.path.join(dir_path, "policy.pt"),
            os.path.join(dir_path, "env.yaml"),
        )

        self.object           = obj
        self.ee_prim          = None
        self.obj_prim         = None
        self._previous_action = np.zeros(8, dtype=np.float32)
        self._policy_counter  = 0

       self.pos_offset_ee = [0.0, 0.2, 0.0]

        self.orn_offset_ee    = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

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
        # read joint states of the robot
        pos_attr = f"{GRAPH_PATH}/{NODE_SUB}.outputs:positionCommand"
        vel_attr = f"{GRAPH_PATH}/{NODE_SUB}.outputs:velocityCommand"
        try:
            joint_positions = og.Controller.attribute(pos_attr).get()
            #print(f"Joint positions: {joint_positions}")
            joint_velocities = og.Controller.attribute(vel_attr).get()
        except Exception as e:
            raise RuntimeError(f"No pude leer JointState desde OmniGraph: {e}")

        if joint_positions is None or joint_velocities is None:
            raise RuntimeError("JointState aún no disponible desde ROS2 bridge")

        
        joint_positions = list(joint_positions)
        joint_velocities = list(joint_velocities)

        if len(joint_positions) == 6:
            joint_positions.insert(6, 0.0)
            joint_velocities.insert(6, 0.0)

        jp = np.pad(np.array(joint_positions, dtype=np.float32), (0, 5), mode='constant')[:12]
        jv = np.pad(np.array(joint_velocities, dtype=np.float32), (0, 5), mode='constant')[:12]

        val_jp = jp[6]
        val_jv = jv[6]

        jp[8] = val_jp
        jp[11] = val_jp

        jv[8] = val_jv
        jv[11] = val_jv

        #print(f"Joint positions: {jp}")
        #print(f"Joint velocities: {jv}")

        obs = np.zeros(42, dtype=np.float32)
        obs[0:12]  = jp[:12] - self.default_pos[:12]
        obs[12:24] = jv[:12] - self.default_vel[:12]

        # object in the scene
        pos_obj, _ = self.object.get_world_pose()
        obs[24:27] = np.array(pos_obj, dtype=np.float32)

        #objective
        goal_pos  = (0.5, 0.5, 0.5)
        goal_quat = (1.0, 0.0, 0.0, 0.0)
        obs[27:30] = np.array(goal_pos, dtype=np.float32)
        obs[30:34] = np.array(goal_quat, dtype=np.float32)

        # last action
        obs[34:42] = self._previous_action
        return obs

    def forward(self, dt: float) -> None:
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            a   = self._compute_action(obs)
            self._previous_action = a.copy()

            pose_raw    = a[:7]
            gripper_raw = float(a[7])

            pos_des     = pose_raw[:3] + self.pos_offset_ee
            quat        = pose_raw[3:7]

            self.desired_pose   = np.concatenate([pos_des, quat]).astype(np.float32)
            self.gripper_action = 1.0 if gripper_raw > 0.0 else 0.0

        self._policy_counter += 1

    def _get_target_command(self) -> np.ndarray:
        raise NotImplementedError("UR5GraspPolicy no implementa _get_target_command")

