# ur5_action_graph.py
import omni.graph.core as og

GRAPH_PATH = "/ActionGraph"
NODE_SUB   = "SubscribeJointState" 

def create_ur5_action_graph_for_joint_state(pub_topic="/joint_states_command", sub_topic="/joint_states"):
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                (NODE_SUB, "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", f"{NODE_SUB}.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", f"{NODE_SUB}.inputs:context"),

                (f"{NODE_SUB}.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (f"{NODE_SUB}.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                (f"{NODE_SUB}.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                (f"{NODE_SUB}.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", "/ur5/root_joint"),
                ("PublishJointState.inputs:targetPrim", "/ur5/root_joint"),
                ("PublishJointState.inputs:topicName", pub_topic),
                (f"{NODE_SUB}.inputs:topicName", sub_topic),
            ],
        },
    )
