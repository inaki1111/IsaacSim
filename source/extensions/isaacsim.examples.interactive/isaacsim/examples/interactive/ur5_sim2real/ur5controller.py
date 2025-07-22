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
        print(f"Articulation action: {art_action}")
        if art_action.joint_positions is None:
            return

        # Publicar a ROS2 vía OmniGraph (PublishJointState)
        jp = art_action.joint_positions
        jv = art_action.joint_velocities

        og.Controller.set(
            "/ActionGraph/PublishJointState.inputs:message",
            {
                "name": "joint_states_command",

                # Lista manual de posiciones, en el orden de tus joints:
                "position": [
                    float(jp[0]),  # shoulder_pan_joint
                    float(jp[1]),  # shoulder_lift_joint
                    float(jp[2]),  # elbow_joint
                    float(jp[3]),  # wrist_2_joint
                    float(jp[4]),  # wrist_3_joint
                    float(jp[5]),  # wrist_1_joint

                ],

                # Lista manual de velocidades, en el mismo orden:
                "velocity": [
                    float(jv[0]),  # shoulder_pan_joint
                    float(jv[1]),  # shoulder_lift_joint
                    float(jv[2]),  # elbow_joint
                    float(jv[3]),  # wrist_2_joint
                    float(jv[4]),  # wrist_3_joint
                    float(jv[5]),  # wrist_1_joint
                ],

                "effort": [],

                "header": {
                    "stamp": {"sec": 0, "nanosec": 0},
                    "frame_id": "",
                },

                # Mantienes self.joint_names si lo necesitas en otro sitio
                "joint_names": self.joint_names,
            }
        )
