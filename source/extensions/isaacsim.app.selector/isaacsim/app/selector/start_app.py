# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
from typing import Any, Dict

import carb.settings
import carb.tokens
import omni.kit.app

from .settings import DEFAULT_APP_SETTING, SHOW_CONSOLE_SETTING


def start_app(
    app_id: str,
    app_version: str,
    app_become_new_default=False,
    persistent_selector=False,
    extra_args=[],
    env=None,
):
    """show the omniverse ui documentation as an external Application"""
    _settings = carb.settings.get_settings()

    # update default
    if app_become_new_default:
        _settings.set(DEFAULT_APP_SETTING, app_id)

    import platform
    import subprocess

    app_folder = _settings.get_as_string("/app/folder")
    if app_folder == "":
        app_folder = carb.tokens.get_tokens_interface().resolve("${app}")

    script_extension = "bat"
    if not sys.platform == "win32":
        script_extension = "sh"

    app_execFile = app_id
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_dict = ext_manager.get_extension_dict(f"{app_id}-{app_version}")
    if ext_dict:
        app_execFile = ext_dict["package"]["execFile"]

    app_start_folder = os.path.normpath(os.path.join(app_folder, os.pardir))
    run_args = [f"{app_start_folder}/{app_execFile}.{script_extension}"]
    run_args.extend(extra_args)

    kwargs: Dict[str, Any] = {"close_fds": False}
    if platform.system().lower() == "windows":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        if _settings.get(SHOW_CONSOLE_SETTING):
            kwargs["creationflags"] |= subprocess.CREATE_NEW_CONSOLE
    else:
        if env:
            run_args = [f'export {k}="{v}";' for k, v in env.items()] + run_args
        if _settings.get(SHOW_CONSOLE_SETTING):
            kwargs["shell"] = True
            run_args = " ".join(run_args)
            run_args = f"x-terminal-emulator -e bash -i -c 'pwd; {run_args}'"
    omni.kit.app.get_app().print_and_log(f"Starting: {run_args}")
    subprocess.Popen(run_args, **kwargs)

    if not persistent_selector:
        omni.kit.app.get_app().post_quit()
