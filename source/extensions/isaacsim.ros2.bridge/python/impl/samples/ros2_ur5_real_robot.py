import asyncio
import gc
import os
import weakref

import carb
import omni.appwindow
import omni.ext
import omni.graph.core as og
import omni.kit.commands
import omni.ui as ui
import usdrt.Sdf
from pxr import Gf

from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import setup_ui_headers
from isaacsim.storage.native import get_assets_root_path

MENU_NAME = "MoveIt"
UR5_STAGE_PATH = "/ur5"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()

        get_browser_instance().register_example(
            name="UR5 ros2", execute_entrypoint=self.build_window, ui_hook=self.build_ui, category="ROS2/MoveIt"
        )

    def build_window(self):
        pass

    def build_ui(self):
        extension_enabled = omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge")
        if not extension_enabled:
            carb.log_error("ROS2 Bridge is not enabled. Please enable the extension to use this feature.")
        else:
            overview = (
                "This sample demonstrates how to receive JointState messages from a real UR5 robot.\n\n"
                "No control will be applied to the simulation. This is for read-only feedback."
            )
            self._main_stack = ui.VStack(spacing=5, height=0)
            with self._main_stack:
                setup_ui_headers(
                    self._ext_id,
                    file_path=os.path.abspath(__file__),
                    title="UR5 ROS2 Listener",
                    overview=overview,
                    info_collapsed=False,
                )
                ui.Button("Load Scene and Start Listening", clicked_fn=self._on_environment_setup)

    def create_ros_action_graph(self, ur5_stage_path):
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeJointState.inputs:topicName", "joint_states"),
                    ],
                },
            )
        except Exception as e:
            carb.log_error(f"Error creating ROS action graph: {e}")

    def create_ur5(self, stage_path):
        usd_path = os.path.join(os.path.dirname(__file__), "ur5_standalone.usdz")

        if not os.path.exists(usd_path):
            carb.log_error(f"UR5 USDZ not found at path: {usd_path}")
            return

        prim = self._stage.DefinePrim(stage_path, "Xform")
        prim.GetReferences().AddReference(usd_path)

        rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=None,
            new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(0, -0.64, 0)),
        )

    async def _create_moveit_sample(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        set_camera_view(eye=[1.20, 1.20, 0.80], target=[0, 0, 0.50], camera_prim_path="/OmniverseKit_Persp")
        self._stage = self._usd_context.get_stage()

        self.create_ur5(UR5_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()

        create_prim(
            prim_path="/background",
            usd_path=self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )

        await omni.kit.app.get_app().next_update_async()
        PhysicsContext(physics_dt=1.0 / 60.0)
        await omni.kit.app.get_app().next_update_async()
        self.create_ros_action_graph(UR5_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

    def _on_environment_setup(self):
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        asyncio.ensure_future(self._create_moveit_sample())

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=MENU_NAME, category="ROS2")
        self._timeline.stop()
        gc.collect()
