# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from typing import Optional, Sequence

import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.preview_surface import PreviewSurface
from isaacsim.core.api.materials.visual_material import VisualMaterial
from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.string import find_unique_string_name
from pxr import Gf, UsdGeom


class VisualCylinder(SingleGeometryPrim):
    """High level wrapper to create/encapsulate a visual cylinder

    .. note::

        Visual cylinders (Cylinder shape) have no collisions (Collider API) or rigid body dynamics (Rigid Body API)

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "visual_cylinder".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        color (Optional[np.ndarray], optional): color of the visual shape. Defaults to None, which means 50% gray
        radius (Optional[float], optional): base radius. Defaults to None.
        height (Optional[float], optional): cylinder height. Defaults to None.
        visual_material (Optional[VisualMaterial], optional): visual material to be applied to the held prim.
                                Defaults to None. If not specified, a default visual material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import VisualCylinder
        >>> import numpy as np
        >>>
        >>> # create a red visual cylinder at the given path
        >>> prim = VisualCylinder(
        ...     prim_path="/World/Xform/Cylinder",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0])
        ... )
        >>> prim
        <isaacsim.core.api.objects.cylinder.VisualCylinder object at 0x7f4e433f22c0>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "visual_cylinder",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        radius: Optional[float] = None,
        height: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
    ) -> None:
        if is_prim_path_valid(prim_path):
            prim = get_prim_at_path(prim_path)
            if not prim.IsA(UsdGeom.Cylinder):
                raise Exception("The prim at path {} cannot be parsed as a Cylinder object".format(prim_path))
            cylinderGeom = UsdGeom.Cylinder(prim)
        else:
            cylinderGeom = UsdGeom.Cylinder.Define(get_current_stage(), prim_path)
            # TODO: double check the capsule extent
            if radius is None:
                radius = 1.0
            if height is None:
                height = 1.0
            if visible is None:
                visible = True
            if visual_material is None:
                if color is None:
                    color = np.array([0.5, 0.5, 0.5])
                visual_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/visual_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
                visual_material = PreviewSurface(prim_path=visual_prim_path, color=color)
        SingleGeometryPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            collision=False,
        )
        if visual_material is not None:
            VisualCylinder.apply_visual_material(self, visual_material)
        if radius is not None:
            VisualCylinder.set_radius(self, radius)
        if height is not None:
            VisualCylinder.set_height(self, height)
        height = VisualCylinder.get_height(self)
        radius = VisualCylinder.get_radius(self)
        cylinderGeom.GetExtentAttr().Set(
            [Gf.Vec3f([-radius, -radius, -height / 2.0]), Gf.Vec3f([radius, radius, height / 2.0])]
        )
        return

    def set_radius(self, radius: float) -> None:
        """Set the base radius

        Args:
            radius (float): base radius

        Example:

        .. code-block:: python

            >>> prim.set_radius(1.0)
        """
        self.geom.GetRadiusAttr().Set(radius)
        return

    def get_radius(self) -> float:
        """Get the base radius

        Returns:
            float: base radius

        Example:

        .. code-block:: python

            >>> prim.get_radius()
            0.5
        """
        return self.geom.GetRadiusAttr().Get()

    def set_height(self, height: float) -> None:
        """Set the cylinder height

        Args:
            height (float): cylinder height

        Example:

        .. code-block:: python

            >>> prim.set_height(2.0)
        """
        self.geom.GetHeightAttr().Set(height)
        return

    def get_height(self) -> float:
        """Get the cylinder height

        Returns:
            float: cylinder height

        Example:

        .. code-block:: python

            >>> prim.get_height()
            1.0
        """
        return self.geom.GetHeightAttr().Get()


class FixedCylinder(VisualCylinder):
    """High level wrapper to create/encapsulate a fixed cylinder

    .. note::

        Fixed cylinders (Cylinder shape) have collisions (Collider API) but no rigid body dynamics (Rigid Body API)

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "fixed_cylinder".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        color (Optional[np.ndarray], optional): color of the visual shape. Defaults to None, which means 50% gray
        radius (Optional[float], optional): base radius. Defaults to None.
        height (Optional[float], optional): cylinder height. Defaults to None.
        visual_material (Optional[VisualMaterial], optional): visual material to be applied to the held prim.
                                Defaults to None. If not specified, a default visual material will be added.
        physics_material (Optional[PhysicsMaterial], optional): physics material to be applied to the held prim.
                                Defaults to None. If not specified, a default physics material will be added.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import FixedCylinder
        >>> import numpy as np
        >>>
        >>> # create a red fixed cylinder at the given path
        >>> prim = FixedCylinder(
        ...     prim_path="/World/Xform/Cylinder",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0])
        ... )
        >>> print(prim)
        <isaacsim.core.api.objects.cylinder.FixedCylinder object at 0x7f4f24144f40>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "fixed_cylinder",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        radius: Optional[np.ndarray] = None,
        height: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
        physics_material: Optional[PhysicsMaterial] = None,
    ) -> None:
        if not is_prim_path_valid(prim_path):
            # set default values if no physics material given
            if physics_material is None:
                static_friction = 0.2
                dynamic_friction = 1.0
                restitution = 0.0
                physics_material_path = find_unique_string_name(
                    initial_name="/World/Physics_Materials/physics_material",
                    is_unique_fn=lambda x: not is_prim_path_valid(x),
                )
                physics_material = PhysicsMaterial(
                    prim_path=physics_material_path,
                    dynamic_friction=dynamic_friction,
                    static_friction=static_friction,
                    restitution=restitution,
                )
        VisualCylinder.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
            height=height,
            visual_material=visual_material,
        )
        SingleGeometryPrim.set_collision_enabled(self, True)
        if physics_material is not None:
            FixedCylinder.apply_physics_material(self, physics_material)
        return


class DynamicCylinder(SingleRigidPrim, FixedCylinder):
    """High level wrapper to create/encapsulate a dynamic cylinder

    .. note::

        Dynamic cylinders (Cylinder shape) have collisions (Collider API) and rigid body dynamics (Rigid Body API)

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "dynamic_cylinder".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        color (Optional[np.ndarray], optional): color of the visual shape. Defaults to None, which means 50% gray
        radius (Optional[float], optional): base radius. Defaults to None.
        height (Optional[float], optional): cylinder height. Defaults to None.
        visual_material (Optional[VisualMaterial], optional): visual material to be applied to the held prim.
                                Defaults to None. If not specified, a default visual material will be added.
        physics_material (Optional[PhysicsMaterial], optional): physics material to be applied to the held prim.
                                Defaults to None. If not specified, a default physics material will be added.
        mass (Optional[float], optional): mass in kg. Defaults to None.
        density (Optional[float], optional): density. Defaults to None.
        linear_velocity (Optional[np.ndarray], optional): linear velocity in the world frame. Defaults to None.
        angular_velocity (Optional[np.ndarray], optional): angular velocity in the world frame. Defaults to None.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.objects import DynamicCylinder
        >>> import numpy as np
        >>>
        >>> # create a red fixed cylinder of mass 1kg at the given path
        >>> prim = DynamicCylinder(
        ...     prim_path="/World/Xform/Cylinder",
        ...     radius=0.5,
        ...     height=1.0,
        ...     color=np.array([1.0, 0.0, 0.0]),
        ...     mass=1.0
        ... )
        >>> prim
        <isaacsim.core.api.objects.cylinder.DynamicCylinder object at 0x7f4e8f5c4a60>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "dynamic_cylinder",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        radius: Optional[np.ndarray] = None,
        height: Optional[np.ndarray] = None,
        visual_material: Optional[VisualMaterial] = None,
        physics_material: Optional[PhysicsMaterial] = None,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        linear_velocity: Optional[Sequence[float]] = None,
        angular_velocity: Optional[Sequence[float]] = None,
    ) -> None:
        if not is_prim_path_valid(prim_path):
            if mass is None:
                mass = 0.02
        FixedCylinder.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
            height=height,
            visual_material=visual_material,
            physics_material=physics_material,
        )
        SingleRigidPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            mass=mass,
            density=density,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
