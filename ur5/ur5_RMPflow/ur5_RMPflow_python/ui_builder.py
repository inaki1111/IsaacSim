import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.objects.cuboid import FixedCuboid, VisualCuboid
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton, Button
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
from isaacsim.core.api.objects.ground_plane import GroundPlane
class _ScenarioStub:
    def setup_scenario(self, articulation, cuboid): pass
    def teardown_scenario(self): pass

class UIBuilder:
    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self.frames = []
        self.wrapped_ui_elements = []
        self.rmpflow = None
        self.articulation_rmpflow = None
        self._on_init()

    def _on_init(self):
        self._articulation = None
        self.cuboid = None
        self._scenario = _ScenarioStub()

    def build_ui(self):
        f = CollapsableFrame("World Controls", collapsed=False)
        with f:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton("Load Button", "LOAD",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._setup_scenario)
                self._load_btn.set_world_settings(physics_dt=1/60.0, rendering_dt=1/60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton("Reset Button", "RESET",
                    pre_reset_fn=None,
                    post_reset_fn=self._on_post_reset_btn)
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        f2 = CollapsableFrame("Run Scenario")
        with f2:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario", "RUN", "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

        f3 = CollapsableFrame("Go To Target")
        with f3:
            Button("Use RMPFlow", "RUN", on_click_fn=self.on_pressed_rmpflow_fn)

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            self._reset_extension()

    def on_physics_step(self, step: float):
        self.on_every_frame()

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def cleanup(self):
        for e in self.wrapped_ui_elements:
            e.cleanup()

    def on_pressed_rmpflow_fn(self):
        if self._articulation is None:
            return
        self.cuboid = VisualCuboid("/cuboid",
            position=np.array([1.0, 0.0, 0.0]),
            size=0.05,
            color=np.array([255, 0, 0]))
        self.rmpflow = RmpFlow(
            robot_description_path="/home/inaki/isaacsim/ur5/ur5_RMPflow/ur5_RMPflow_python/ur5_lula.yaml",
            urdf_path="/home/inaki/isaacsim/ur5/ur5_RMPflow/ur5_RMPflow_python/ur5.urdf",
            rmpflow_config_path="/home/inaki/isaacsim/ur5/ur5_RMPflow/ur5_RMPflow_python/rmpflow.yaml",
            end_effector_frame_name="robotiq_85_base_link",
            maximum_substep_size=0.0034
        )
        self.articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self.rmpflow, 1/60)

    def on_every_frame(self):
        if self.rmpflow is None:
            return
        self.rmpflow.set_end_effector_target(
            target_position=self.cuboid.get_world_pose()[0]
            #target_orientation=self.cuboid.get_world_pose()[1],
        )

        action = self.articulation_rmpflow.get_next_articulation_action()


        self._articulation.get_articulation_controller().set_gains(1e15*np.ones(12), 1e14*np.ones(12))
        self._articulation.apply_action(action)

    def _setup_scene(self):
        robot_path = "/ur5"
        usd_path = "/home/inaki/isaacsim/ur5/ur5_RMPflow/ur5_RMPflow_python/ur5_standalone.usd"
        create_new_stage()
        UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight")) \
            .CreateIntensityAttr(100000)
        add_reference_to_stage(usd_path, robot_path)
        self._articulation = SingleArticulation(robot_path)
        
        w = World.instance()
        w.scene.add(self._articulation)

    def _setup_scenario(self):
        self._scenario.teardown_scenario()
        self._scenario.setup_scenario(self._articulation, self.cuboid)
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        self._scenario.teardown_scenario()
        self._scenario.setup_scenario(self._articulation, self.cuboid)
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        self._scenario.update_scenario(step)

    def _on_run_scenario_a_text(self):
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        self._timeline.pause()

    def _reset_extension(self):
        self._on_init()
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
