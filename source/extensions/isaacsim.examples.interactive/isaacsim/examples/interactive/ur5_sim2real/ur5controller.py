import numpy as np
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
import omni.graph.core as og
import os

class UR5Controller:
    def __init__(self, robot, joint_names):
        self.robot = robot
        self.joint_names = joint_names

        dir = os.path.dirname(__file__)
        self.rmpflow = RmpFlow(
            robot_description_path=os.path.join(dir, "ur5_lula.yaml"),
            urdf_path=os.path.join(dir, "ur5.urdf"),
            rmpflow_config_path=os.path.join(dir, "rmpflow.yaml"),
            end_effector_frame_name="wrist_3_link",
            maximum_substep_size=0.0034,
        )

        self.motion_policy = ArticulationMotionPolicy(self.robot, self.rmpflow, 1.0 / 400.0)
        self.body_offset = np.array([0.0, 0.1, 0.0], dtype=np.float32)

    def apply_action(self, action):
        pose_raw = np.asarray(action[0], dtype=np.float32)
        target_pos = pose_raw[:3]
        target_quat = pose_raw[3:7]

        self.rmpflow.set_end_effector_target(
            target_position=target_pos,
            target_orientation=target_quat,
        )

        art_action = self.motion_policy.get_next_articulation_action()
        if art_action.joint_positions is None:
            return

        # Publicar a ROS2 vía OmniGraph (PublishJointState)
        og.Controller.set(
            "/ActionGraph/PublishJointState.inputs:message",
            {
                "name": "joint_states_command",
                "position": art_action.joint_positions.tolist(),
                "velocity": [],
                "effort": [],
                "header": {
                    "stamp": {"sec": 0, "nanosec": 0},
                    "frame_id": "",
                },
                "joint_names": self.joint_names,
            }
        )
