import numpy as np
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
import os
class UR5Controller:
    def __init__(self, robot):
        self.robot = robot
        dir = os.path.dirname(__file__)
        # rmp flow files 
        """self.rmpflow = RmpFlow(
            robot_description_path="/home/inaki/IsaacSim/ur5/ur5_RMPflow/ur5_RMPflow_python/ur5_lula.yaml",
            urdf_path="/home/inaki/IsaacSim/ur5/ur5_RMPflow/ur5_RMPflow_python/ur5.urdf",
            rmpflow_config_path="/home/inaki/IsaacSim/ur5/ur5_RMPflow/ur5_RMPflow_python/rmpflow.yaml",
            end_effector_frame_name="wrist_3_link",
            maximum_substep_size=0.0034,
        )"""


        # rmp flow files 
        self.rmpflow = RmpFlow(
            robot_description_path=os.path.join(dir, "ur5_lula.yaml"),
            urdf_path=os.path.join(dir, "ur5.urdf"),
            rmpflow_config_path=os.path.join(dir, "rmpflow.yaml"),
            end_effector_frame_name="wrist_3_link",
            maximum_substep_size=0.0034,
        )


        self.motion_policy = ArticulationMotionPolicy(self.robot, self.rmpflow, 1.0/400.0)

        # offset of the gripper
        self.body_offset = np.array([0.0, 0.1, 0.0], dtype=np.float32)

    def apply_action(self, action):
        """
        action: tuple (pose_raw, gripper_raw)
          - pose_raw: array de 7 floats en world-frame [x,y,z,qx,qy,qz,qw]
          - gripper_raw: float en [-1,1]
        """

        pose_raw    = np.asarray(action[0], dtype=np.float32)

        gripper_raw = float(action[1])

        # target position
        #target_pos  = pose_raw[:3] + self.body_offset
        target_pos  = pose_raw[:3]

        target_quat = np.asarray(pose_raw[3:7], dtype=np.float32)

        # 4) send pose to rmpflow
        self.rmpflow.set_end_effector_target(
            target_position=target_pos.astype(np.float32),
            target_orientation=target_quat.astype(np.float32),
        )

        # apply the joint action
        art_action = self.motion_policy.get_next_articulation_action()
        jp = self.robot.get_joint_positions()
        if jp is None:
            return
        n = len(jp)

        gains = 20000.0 * np.ones(n, dtype=np.float32)
        damp  =  1000.0 * np.ones(n, dtype=np.float32)
        self.robot.get_articulation_controller().set_gains(gains, damp)
        self.robot.apply_action(art_action)


        # 7) Debug
        #print(f"[Controller] target_pos={target_pos}, quat={target_quat}, gripper_cmd={gripper_raw}")
