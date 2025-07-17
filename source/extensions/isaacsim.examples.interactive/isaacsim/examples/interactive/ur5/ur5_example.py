import numpy as np
import omni
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.examples.interactive.base_sample import BaseSample
from .ur5_policy import UR5GraspPolicy
from .ur5controller import UR5Controller  


class UR5GraspExample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 400.0
        self._world_settings["rendering_dt"] = 1.0 / 60.0

    def setup_scene(self) -> None:
        self.get_world().scene.add_default_ground_plane(
            z_position=-0.63,
            name="ground_plane",
            prim_path="/World/groundPlane",
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        )

        table_prim = "/World/Table"
        table_usd = "/home/inaki/isaacsim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur5/table.usd"
        add_reference_to_stage(table_usd, table_prim)
        self.table = SingleArticulation(
            prim_path=table_prim,
            name="Table",
            position=np.array([0.5, 0.0, -0.63]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        obj_prim = "/World/Object"
        cube_usd = (
            "http://omniverse-content-production.s3-us-west-2.amazonaws.com"
            "/Assets/Isaac/4.5/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd"
        )
        add_reference_to_stage(cube_usd, obj_prim)
        self.object = SingleArticulation(
            prim_path=obj_prim,
            name="object",
            position=np.array([0.5, 0.0, 0.03]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        # policy controller
        self.policy = UR5GraspPolicy(
            prim_path="/World/ur5",
            obj=self.object,
            name="ur5_grasp",
        )
        self.controller = UR5Controller(self.policy.robot)

        timeline = omni.timeline.get_timeline_interface()
        self._timer_sub = (
            timeline.get_timeline_event_stream()
            .create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP),
                self._on_timeline_stop,
            )
        )

    async def setup_post_load(self) -> None:
        self._physics_ready = False
        self.get_world().add_physics_callback(
            "physics_step", callback_fn=self._on_physics
        )
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        self.policy._previous_action = np.zeros(8, dtype=np.float32)
        await self.get_world().play_async()

    def _on_physics(self, step_size: float) -> None:
        if self._physics_ready:
            self.policy.forward(step_size)
            action = self.policy.desired_pose, self.policy.gripper_action
            self.controller.apply_action(action)
        else:
            self._physics_ready = True
            self.policy.initialize()
            self.policy.post_reset()
            self.policy.robot.set_joints_default_state(self.policy.default_pos)

    def _on_timeline_stop(self, event) -> None:
        self._physics_ready = False

    def world_cleanup(self) -> None:
        world = self.get_world()
        if world.physics_callback_exists("physics_step"):
            world.remove_physics_callback("physics_step")
        self._timer_sub = None
