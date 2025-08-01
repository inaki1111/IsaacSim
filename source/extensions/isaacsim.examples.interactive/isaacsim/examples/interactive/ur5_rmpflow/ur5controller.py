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
        # define target position and orientation for the end effector
        target_pos  = np.array([-0.16810509302823873, 0.5859542808301269, 0.26483031099633514], dtype=np.float32)
        target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # add the target to the RMPFlow
        self.rmpflow.set_end_effector_target(
            target_position=target_pos,
            target_orientation=target_quat,
        )

        # obtain the next articulation action from the motion policy
        art_action = self.motion_policy.get_next_articulation_action()
        print(f"Articulation action: {art_action}")
        if art_action.joint_positions is None:
            return

        #  OmniGraph (PublishJointState)
        jp = art_action.joint_positions
        jv = art_action.joint_velocities

        og.Controller.set(
            "/ActionGraph/PublishJointState.inputs:message",
            {
                "name": "joint_states_command",
                "position": [
                    float(jp[0]),  # shoulder_pan_joint
                    float(jp[1]),  # shoulder_lift_joint
                    float(jp[2]),  # elbow_joint
                    float(jp[3]),  # wrist_2_joint
                    float(jp[4]),  # wrist_3_joint
                    float(jp[5]),  # wrist_1_joint
                ],
                "velocity": [
                    float(jv[0]),
                    float(jv[1]),
                    float(jv[2]),
                    float(jv[3]),
                    float(jv[4]),
                    float(jv[5]),
                ],
                "effort": [],
                "header": {
                    "stamp": {"sec": 0, "nanosec": 0},
                    "frame_id": "",
                },
                "joint_names": self.joint_names,
            }
        )
