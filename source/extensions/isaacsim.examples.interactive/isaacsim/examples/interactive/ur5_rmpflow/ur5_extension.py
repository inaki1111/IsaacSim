import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.examples.interactive.ur5_rmpflow.ur5_example import UR5GraspExample


class UR5ExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):

        self.example_name = "UR5_rmpflow"
        self.category = "Policy"

        overview = "UR5 policy to lift dynamic objects"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Manipulator: UR5",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html",
            "overview": overview,
            "sample": UR5GraspExample(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)

        return
