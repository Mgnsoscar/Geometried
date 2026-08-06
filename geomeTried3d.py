"""
geomeTried3d.py

3D solid primitives built on trimesh, mirroring the API of the shapely-based
`Shape` classes in geomeTried2d.py.

Conventions
-----------
* Every primitive is built centred on the origin and then moved by `pos`.
* Transform methods mutate in place and return `self`, so the concrete subclass
  is preserved and calls chain: ``Box(10, 4, 2).rotate(30).min_to("z", 0)``.
* Boolean operations and `set_width` return a new, generic `Structure`.
* Angles are in degrees.
* Shape-specific properties (`radius`, `width`, ...) are derived from the
  geometry rather than stored, so they stay correct after scaling.
* No units library and no Vector type - plain floats and numpy arrays.

The three polygon constructors (`extrude_polygon`, `sweep_polygon`,
`rotate_polygon`) take any 2D profile: a geomeTried2d `Shape`, a shapely
Polygon, or an (N, 2) vertex array. The 2D module is duck-typed via its `.geom`
attribute, so this file does not import it.

Optional dependencies
---------------------
* manifold3d - boolean operations (union / difference / intersection, holes)
* rtree      - `contains`
* python-fcl - `intersects_with`, which otherwise falls back to a boolean test
"""

from __future__ import annotations

import typing
from typing import Self

import numpy as np
import shapely
import shapely.geometry.polygon
import trimesh
from trimesh import Trimesh, Scene, boolean, creation
from trimesh.transformations import rotation_matrix
from trimesh.visual import ColorVisuals

Axis = typing.Literal["x", "y", "z"]
Extremity = typing.Literal["min", "max"]
ArrayLike = np.typing.ArrayLike
StructureLike = typing.Union["Structure", Trimesh, ArrayLike]
PolygonLike = typing.Any  # Shape, shapely Polygon, or (N, 2) array

__all__ = [
    "Structure",
    "Box",
    "Cylinder",
    "Sphere",
    "Cone",
    "Pyramid",
]


class Structure:
    """
    A 3D solid backed by a trimesh Trimesh.
    """

    _mesh: Trimesh

    def __init__(
            self,
            mesh: StructureLike,
            holes: typing.Sequence[StructureLike] | None = None,
    ) -> None:
        geom = self._coerce(mesh)

        if holes:
            cutter = boolean.union([self._coerce(h).copy() for h in holes])
            geom = boolean.difference([geom.copy(), cutter])

        if geom.is_empty:
            raise ValueError("Resulting mesh is empty.")

        self._mesh = geom

    # ------------------------------------------------------------------ #
    # construction helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _coerce(mesh: StructureLike) -> Trimesh:
        """
        Accept a Structure, a Trimesh, a (vertices, faces) pair, or an (N, 3)
        point array - the last is wrapped in its convex hull.
        """
        if isinstance(mesh, Structure):
            return mesh._mesh
        if isinstance(mesh, Trimesh):
            return mesh

        if isinstance(mesh, tuple) and len(mesh) == 2:
            vertices, faces = mesh
            return Trimesh(vertices=np.asarray(vertices, dtype=float),
                           faces=np.asarray(faces, dtype=np.int64))

        points = np.asarray(mesh, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise TypeError(
                f"Expected a Structure, a Trimesh, a (vertices, faces) pair, or "
                f"an (N, 3) point array; got an array of shape {points.shape}."
            )
        if len(points) < 4:
            raise ValueError("At least four points are required for a solid.")

        return trimesh.convex.convex_hull(points)

    @staticmethod
    def _as_polygon(polygon: PolygonLike) -> shapely.Polygon:
        """Accept a geomeTried2d Shape, a shapely Polygon, or an (N, 2) array."""
        geom = getattr(polygon, "geom", polygon)

        if isinstance(geom, shapely.Polygon):
            return geom
        if isinstance(geom, shapely.MultiPolygon):
            raise ValueError(
                "Profile has multiple disjoint parts; build each one separately."
            )

        vertices = np.asarray(geom, dtype=float)
        if vertices.ndim != 2 or vertices.shape[1] != 2:
            raise TypeError(
                f"Expected a 2D Shape, a shapely Polygon, or an (N, 2) vertex "
                f"array; got an array of shape {vertices.shape}."
            )
        return shapely.Polygon(vertices)

    def copy(self) -> Self:
        new = object.__new__(type(self))
        new._mesh = self._mesh.copy()
        return new

    # ------------------------------------------------------------------ #
    # polygon constructors
    # ------------------------------------------------------------------ #

    @classmethod
    def extrude_polygon(
            cls,
            polygon: PolygonLike,
            height: float,
            centred: bool = True,
            **kwargs,
    ) -> Structure:
        """
        Extrude a 2D profile along +z. Holes in the profile are carried through.

        The profile keeps its x/y position. With `centred` the solid straddles
        z = 0; otherwise it runs from z = 0 to z = height.
        """
        return Structure(creation.extrude_polygon(
            cls._as_polygon(polygon), height, mid_plane=centred, **kwargs
        ))

    @classmethod
    def sweep_polygon(
            cls,
            polygon: PolygonLike,
            path: ArrayLike,
            angles: ArrayLike | None = None,
            cap: bool = True,
            **kwargs,
    ) -> Structure:
        """
        Sweep a 2D profile along a 3D path given as an (N, 3) array of points.

        `angles` optionally twists the profile, in radians, one value per path
        point.
        """
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 3:
            raise ValueError(
                f"Path must be an (N, 3) array of points; got shape {path.shape}."
            )

        return Structure(creation.sweep_polygon(
            cls._as_polygon(polygon), path, angles=angles, cap=cap, **kwargs
        ))

    @classmethod
    def rotate_polygon(
            cls,
            polygon: PolygonLike,
            angle: float = 360.0,
            sections: int = 64,
            cap: bool | None = None,
            **kwargs,
    ) -> Structure:
        """
        Revolve a 2D profile about the z axis to make a solid of revolution.

        The profile lies in the xz plane in the sense that its x is radius and
        its y becomes height, so it must not cross into negative x. `angle` is
        the sweep in degrees; a partial sweep is capped at both ends by default.
        """
        profile = cls._as_polygon(polygon)

        if profile.interiors:
            raise ValueError(
                "A revolved profile cannot have holes; revolve the outline and "
                "subtract the inner solid instead."
            )
        if profile.bounds[0] < -1e-12:
            raise ValueError(
                f"Profile crosses the axis of revolution: its minimum x is "
                f"{profile.bounds[0]:.6g}, which would give a negative radius."
            )

        partial = not np.isclose(angle % 360, 0)
        if cap is None:
            cap = partial

        # The revolved surface takes its normal direction from the profile's
        # winding, and shapely preserves whatever order the profile was built
        # with - a clockwise one yields an inside-out mesh with negative volume.
        profile = shapely.geometry.polygon.orient(profile, sign=1.0)

        mesh = creation.revolve(
            np.asarray(profile.exterior.coords),
            angle=np.radians(angle) if partial else None,
            sections=sections,
            cap=cap,
            **kwargs,
        )

        # Vertices sitting on the axis revolve into a fan of coincident points
        # and zero-area faces, which leaves the mesh technically open.
        mesh.merge_vertices()
        mesh.update_faces(mesh.nondegenerate_faces())
        mesh.remove_unreferenced_vertices()
        mesh.fix_normals()

        if (cap or not partial) and not mesh.is_volume:
            raise ValueError(
                "Revolving this profile did not produce a closed solid "
                f"(watertight={mesh.is_watertight}, "
                f"winding_consistent={mesh.is_winding_consistent}, "
                f"bodies={mesh.body_count}). Check that the profile is a simple, "
                "non-self-intersecting ring."
            )

        return Structure(mesh)

    # ------------------------------------------------------------------ #
    # geometry access
    # ------------------------------------------------------------------ #

    @property
    def mesh(self) -> Trimesh:
        return self._mesh

    @property
    def vertices(self) -> np.ndarray:
        return self._mesh.vertices

    @property
    def faces(self) -> np.ndarray:
        return self._mesh.faces

    @property
    def is_volume(self) -> bool:
        return self._mesh.is_volume

    @property
    def is_watertight(self) -> bool:
        return self._mesh.is_watertight

    @property
    def volume(self) -> float:
        return self._mesh.volume

    @property
    def area(self) -> float:
        return self._mesh.area

    @property
    def bounds(self) -> np.ndarray:
        return self._mesh.bounds

    @property
    def body_count(self) -> int:
        return self._mesh.body_count

    # ------------------------------------------------------------------ #
    # position and extent
    # ------------------------------------------------------------------ #

    @property
    def centroid(self) -> np.ndarray:
        """
        Centre of volume for a watertight solid, falling back to the area
        weighted surface centroid when the mesh is not a volume. These agree for
        every primitive in this module; see `center_mass` / `surface_centroid`
        to pin down one or the other.
        """
        return self._mesh.center_mass if self._mesh.is_volume else self._mesh.centroid

    @property
    def center_mass(self) -> np.ndarray:
        return self._mesh.center_mass

    @property
    def surface_centroid(self) -> np.ndarray:
        return self._mesh.centroid

    @property
    def x(self) -> float:
        return self.centroid[0]

    @property
    def y(self) -> float:
        return self.centroid[1]

    @property
    def z(self) -> float:
        return self.centroid[2]

    def _get_extremity(self, axis: Axis, type: Extremity = "min") -> float:
        idx = ["x", "y", "z"].index(axis)
        return float(self._mesh.bounds[0 if type == "min" else 1][idx])

    def _get_span(self, axis: Axis) -> float:
        return getattr(self, f"{axis}max") - getattr(self, f"{axis}min")

    @property
    def xmin(self) -> float:
        return self._get_extremity("x", "min")

    @property
    def xmax(self) -> float:
        return self._get_extremity("x", "max")

    @property
    def ymin(self) -> float:
        return self._get_extremity("y", "min")

    @property
    def ymax(self) -> float:
        return self._get_extremity("y", "max")

    @property
    def zmin(self) -> float:
        return self._get_extremity("z", "min")

    @property
    def zmax(self) -> float:
        return self._get_extremity("z", "max")

    @property
    def xspan(self) -> float:
        return self._get_span("x")

    @property
    def yspan(self) -> float:
        return self._get_span("y")

    @property
    def zspan(self) -> float:
        return self._get_span("z")

    @property
    def bbox_center(self) -> np.ndarray:
        return self._mesh.bounds.mean(axis=0)

    # -- the eight bounding box corners --------------------------------- #

    def _corner(self, xs: Extremity, ys: Extremity, zs: Extremity) -> np.ndarray:
        return np.array([
            self._get_extremity("x", xs),
            self._get_extremity("y", ys),
            self._get_extremity("z", zs),
        ])

    @property
    def xminyminzmin(self) -> np.ndarray:
        return self._corner("min", "min", "min")

    @property
    def xminyminzmax(self) -> np.ndarray:
        return self._corner("min", "min", "max")

    @property
    def xminymaxzmin(self) -> np.ndarray:
        return self._corner("min", "max", "min")

    @property
    def xminymaxzmax(self) -> np.ndarray:
        return self._corner("min", "max", "max")

    @property
    def xmaxyminzmin(self) -> np.ndarray:
        return self._corner("max", "min", "min")

    @property
    def xmaxyminzmax(self) -> np.ndarray:
        return self._corner("max", "min", "max")

    @property
    def xmaxymaxzmin(self) -> np.ndarray:
        return self._corner("max", "max", "min")

    @property
    def xmaxymaxzmax(self) -> np.ndarray:
        return self._corner("max", "max", "max")

    @property
    def corners(self) -> np.ndarray:
        """All eight corners as an (8, 3) array."""
        return np.array([
            self._corner(xs, ys, zs)
            for xs in ("min", "max")
            for ys in ("min", "max")
            for zs in ("min", "max")
        ])

    # -- the six bounding box face centres ------------------------------ #

    def _face_center(self, axis: Axis, type: Extremity) -> np.ndarray:
        centre = self.bbox_center.copy()
        centre[["x", "y", "z"].index(axis)] = self._get_extremity(axis, type)
        return centre

    @property
    def xmincenter(self) -> np.ndarray:
        return self._face_center("x", "min")

    @property
    def xmaxcenter(self) -> np.ndarray:
        return self._face_center("x", "max")

    @property
    def ymincenter(self) -> np.ndarray:
        return self._face_center("y", "min")

    @property
    def ymaxcenter(self) -> np.ndarray:
        return self._face_center("y", "max")

    @property
    def zmincenter(self) -> np.ndarray:
        return self._face_center("z", "min")

    @property
    def zmaxcenter(self) -> np.ndarray:
        return self._face_center("z", "max")

    @property
    def face_centers(self) -> np.ndarray:
        """All six face centres as a (6, 3) array."""
        return np.array([
            self._face_center(axis, type)
            for axis in ("x", "y", "z")
            for type in ("min", "max")
        ])

    # ------------------------------------------------------------------ #
    # projection onto an arbitrary axis
    # ------------------------------------------------------------------ #

    @staticmethod
    def _unit(vector: ArrayLike) -> np.ndarray:
        v = np.asarray(vector, dtype=float).ravel()[:3]
        norm = np.linalg.norm(v)
        if np.isclose(norm, 0):
            raise ValueError("Axis vector has zero length.")
        return v / norm

    def min_along(self, vector: ArrayLike) -> float:
        """Smallest coordinate of the solid projected onto `vector`."""
        return float(np.min(self._mesh.vertices @ self._unit(vector)))

    def max_along(self, vector: ArrayLike) -> float:
        """Largest coordinate of the solid projected onto `vector`."""
        return float(np.max(self._mesh.vertices @ self._unit(vector)))

    def span_along(self, vector: ArrayLike) -> float:
        projection = self._mesh.vertices @ self._unit(vector)
        return float(np.max(projection) - np.min(projection))

    # ------------------------------------------------------------------ #
    # transforms - return the same type
    # ------------------------------------------------------------------ #

    def translate(self, vector: ArrayLike) -> Self:
        self._mesh.apply_translation(np.asarray(vector, dtype=float).ravel()[:3])
        return self

    def set_position(self, pos: ArrayLike) -> Self:
        return self.translate(np.asarray(pos, dtype=float).ravel()[:3] - self.centroid)

    def rotate(
            self,
            angle: float,
            axis: ArrayLike = (0, 0, 1),
            point: ArrayLike | None = None,
    ) -> Self:
        """
        Rotate by `angle` degrees about `axis`, through the centroid unless
        `point` is given.
        """
        centre = self.centroid if point is None else np.asarray(point, dtype=float)
        self._mesh.apply_transform(
            rotation_matrix(np.radians(angle), self._unit(axis), centre)
        )
        return self

    def scale(
            self,
            factor: float,
            factor_y: float | None = None,
            factor_z: float | None = None,
            point: ArrayLike | None = None,
    ) -> Self:
        """
        Scale about the centroid unless `point` is given. Pass `factor_y` and
        `factor_z` for a non-uniform scale.
        """
        factors = np.array([
            factor,
            factor if factor_y is None else factor_y,
            factor if factor_z is None else factor_z,
        ], dtype=float)
        centre = self.centroid if point is None else np.asarray(point, dtype=float)

        self._mesh.apply_transform(self._scale_matrix(factors, centre))
        return self

    @staticmethod
    def _scale_matrix(factors: ArrayLike, point: ArrayLike) -> np.ndarray:
        factors = np.asarray(factors, dtype=float)
        point = np.asarray(point, dtype=float)

        matrix = np.eye(4)
        matrix[:3, :3] = np.diag(factors)
        matrix[:3, 3] = point - factors * point
        return matrix

    def min_to(self, axis: Axis, pos: float, buffer: float = 0.0) -> Self:
        delta = (pos - getattr(self, f"{axis}min")) + buffer
        vector = np.zeros(3)
        vector[["x", "y", "z"].index(axis)] = delta
        return self.translate(vector)

    def max_to(self, axis: Axis, pos: float, buffer: float = 0.0) -> Self:
        delta = (pos - getattr(self, f"{axis}max")) + buffer
        vector = np.zeros(3)
        vector[["x", "y", "z"].index(axis)] = delta
        return self.translate(vector)

    def min_along_to(self, axis: ArrayLike, pos: float, buffer: float = 0.0) -> Self:
        unit = self._unit(axis)
        return self.translate(unit * ((pos - self.min_along(unit)) + buffer))

    def max_along_to(self, axis: ArrayLike, pos: float, buffer: float = 0.0) -> Self:
        unit = self._unit(axis)
        return self.translate(unit * ((pos - self.max_along(unit)) + buffer))

    # ------------------------------------------------------------------ #
    # operations - return a generic Structure
    # ------------------------------------------------------------------ #

    def union(self, *structures: StructureLike) -> Structure:
        return Structure(boolean.union(
            [self._mesh.copy(), *(self._coerce(s).copy() for s in structures)]
        ))

    def intersection(self, *structures: StructureLike) -> Structure:
        return Structure(boolean.intersection(
            [self._mesh.copy(), *(self._coerce(s).copy() for s in structures)]
        ))

    def difference(self, *structures: StructureLike) -> Structure:
        if not structures:
            return Structure(self._mesh.copy())
        cutter = boolean.union([self._coerce(s).copy() for s in structures])
        return Structure(boolean.difference([self._mesh.copy(), cutter]))

    def set_width(self, axis: Axis, width: float, point: float | None = None) -> Structure:
        """
        Scale along `axis` so that the span becomes `width`, holding the plane at
        `point` fixed. Defaults to the centroid coordinate on that axis.
        """
        span = getattr(self, f"{axis}span")
        if np.isclose(span, 0):
            raise ValueError(
                f"Cannot set width along {axis!r}: current {axis}span is zero."
            )

        if point is None:
            point = getattr(self, axis)

        idx = ["x", "y", "z"].index(axis)
        factors = np.ones(3)
        factors[idx] = width / span
        anchor = np.zeros(3)
        anchor[idx] = point

        mesh = self._mesh.copy()
        mesh.apply_transform(self._scale_matrix(factors, anchor))
        return Structure(mesh)

    def convex_hull(self, *structures: StructureLike) -> Structure:
        points = np.vstack([
            self._mesh.vertices,
            *(self._coerce(s).vertices for s in structures),
        ])
        return Structure(trimesh.convex.convex_hull(points))

    # ------------------------------------------------------------------ #
    # queries and display
    # ------------------------------------------------------------------ #

    def intersects_with(self, *structures: StructureLike) -> bool:
        """
        True if any of `structures` overlaps this one.

        Uses FCL when the optional python-fcl package is installed, and
        otherwise falls back to a boolean intersection, which is slower but
        needs no extra dependency.
        """
        meshes = [self._coerce(s) for s in structures]

        try:
            manager = trimesh.collision.CollisionManager()
            for i, mesh in enumerate(meshes):
                manager.add_object(f"mesh{i}", mesh)
            return manager.in_collision_single(self._mesh)
        except ValueError:
            return any(
                boolean.intersection([self._mesh.copy(), mesh.copy()]).volume > 1e-9
                for mesh in meshes
            )

    def contains(self, points: ArrayLike) -> np.ndarray:
        return self._mesh.contains(np.atleast_2d(np.asarray(points, dtype=float)))

    def set_color(self, r: int, g: int, b: int, a: int = 255) -> Self:
        self._mesh.visual = ColorVisuals(mesh=self._mesh, face_colors=[r, g, b, a])
        return self

    def show(self, *structures: Structure):
        scene = Scene((self._mesh, *(s._mesh for s in structures)))
        scene.camera.z_far = 1e6
        return scene.show()

    def export(self, filename: str) -> None:
        self._mesh.export(filename)

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(x={self.x:.4g}, y={self.y:.4g}, z={self.z:.4g}, "
            f"xspan={self.xspan:.4g}, yspan={self.yspan:.4g}, zspan={self.zspan:.4g})"
        )


# ====================================================================== #
# primitives
# ====================================================================== #


class Box(Structure):
    """
    Axis-aligned box. `width`, `depth` and `height` are the x, y and z extents.
    """

    def __init__(
            self,
            width: float,
            depth: float,
            height: float,
            pos: ArrayLike = (0, 0, 0),
    ):
        if min(width, depth, height) <= 0:
            raise ValueError("Box extents must all be positive.")

        super().__init__(creation.box(extents=(width, depth, height)))
        self.translate(pos)

    @classmethod
    def cube(cls, side: float, pos: ArrayLike = (0, 0, 0)) -> Self:
        return cls(side, side, side, pos=pos)

    @classmethod
    def from_bounds(cls, bounds: ArrayLike) -> Self:
        """Build from a trimesh-style ((xmin, ymin, zmin), (xmax, ymax, zmax))."""
        low, high = np.asarray(bounds, dtype=float)
        return cls(*(high - low), pos=(low + high) / 2)

    @property
    def width(self) -> float:
        return self.xspan

    @property
    def depth(self) -> float:
        return self.yspan

    @property
    def height(self) -> float:
        return self.zspan

    @property
    def diagonal(self) -> float:
        return float(np.linalg.norm([self.width, self.depth, self.height]))


class Cylinder(Structure):
    """
    Cylinder along z, elliptical when `radius_y` differs from `radius`.
    """

    def __init__(
            self,
            radius: float,
            height: float,
            radius_y: float | None = None,
            sections: int = 64,
            pos: ArrayLike = (0, 0, 0),
    ):
        if radius <= 0 or height <= 0:
            raise ValueError("radius and height must be positive.")
        if sections < 3:
            raise ValueError("sections must be at least 3.")

        ry = radius if radius_y is None else radius_y
        if ry <= 0:
            raise ValueError("radius_y must be positive.")

        mesh = creation.cylinder(radius=radius, height=height, sections=sections)
        if not np.isclose(ry, radius):
            mesh.apply_scale([1.0, ry / radius, 1.0])

        super().__init__(mesh)
        self.translate(pos)

    @classmethod
    def from_diameter(cls, diameter: float, height: float, **kwargs) -> Self:
        return cls(diameter / 2, height, **kwargs)

    @property
    def radius_x(self) -> float:
        return self.xspan / 2

    @property
    def radius_y(self) -> float:
        return self.yspan / 2

    @property
    def radius(self) -> float:
        if not np.isclose(self.radius_x, self.radius_y):
            raise ValueError(
                "Cross-section is elliptical; use `radius_x` / `radius_y` instead."
            )
        return self.radius_x

    @property
    def height(self) -> float:
        return self.zspan

    @property
    def is_elliptical(self) -> bool:
        return not np.isclose(self.radius_x, self.radius_y)


class Sphere(Structure):
    """
    Sphere, or an ellipsoid when `radius_y` or `radius_z` are given.
    """

    def __init__(
            self,
            radius: float,
            radius_y: float | None = None,
            radius_z: float | None = None,
            subdivisions: int = 3,
            pos: ArrayLike = (0, 0, 0),
    ):
        if radius <= 0:
            raise ValueError("radius must be positive.")

        ry = radius if radius_y is None else radius_y
        rz = radius if radius_z is None else radius_z
        if min(ry, rz) <= 0:
            raise ValueError("radius_y and radius_z must be positive.")

        mesh = creation.icosphere(subdivisions=subdivisions, radius=radius)
        if not (np.isclose(ry, radius) and np.isclose(rz, radius)):
            mesh.apply_scale([1.0, ry / radius, rz / radius])

        super().__init__(mesh)
        self.translate(pos)

    @classmethod
    def from_diameter(cls, diameter: float, **kwargs) -> Self:
        return cls(diameter / 2, **kwargs)

    @property
    def radius_x(self) -> float:
        return self.xspan / 2

    @property
    def radius_y(self) -> float:
        return self.yspan / 2

    @property
    def radius_z(self) -> float:
        return self.zspan / 2

    @property
    def radii(self) -> np.ndarray:
        return np.array([self.radius_x, self.radius_y, self.radius_z])

    @property
    def radius(self) -> float:
        radii = self.radii
        if not np.allclose(radii, radii[0]):
            raise ValueError(
                "Shape is an ellipsoid; use `radii` or `radius_x/y/z` instead."
            )
        return float(radii[0])

    @property
    def is_ellipsoid(self) -> bool:
        radii = self.radii
        return not np.allclose(radii, radii[0])


class Cone(Structure):
    """
    Cone along z. A non-zero `radius_top` gives a truncated cone (frustum),
    with `radius` the radius at the bottom.
    """

    def __init__(
            self,
            radius: float,
            height: float,
            radius_top: float = 0.0,
            sections: int = 64,
            pos: ArrayLike = (0, 0, 0),
    ):
        if radius <= 0 or height <= 0:
            raise ValueError("radius and height must be positive.")
        if radius_top < 0:
            raise ValueError("radius_top cannot be negative.")
        if sections < 3:
            raise ValueError("sections must be at least 3.")

        profile = [(0.0, 0.0), (radius, 0.0)]
        if radius_top > 0:
            profile.append((radius_top, height))
        profile.append((0.0, height))

        mesh = creation.revolve(
            np.asarray(profile + [profile[0]], dtype=float), sections=sections
        )
        mesh.apply_translation([0, 0, -height / 2])

        super().__init__(mesh)
        self.translate(pos)

    @classmethod
    def frustum(
            cls, radius_bottom: float, radius_top: float, height: float, **kwargs
    ) -> Self:
        return cls(radius_bottom, height, radius_top=radius_top, **kwargs)

    @property
    def radius(self) -> float:
        """Radius at the bottom face."""
        section = self._mesh.vertices[
            np.isclose(self._mesh.vertices[:, 2], self.zmin)
        ]
        return float(np.max(np.linalg.norm(section[:, :2] - self.centroid[:2], axis=1)))

    @property
    def radius_top(self) -> float:
        section = self._mesh.vertices[
            np.isclose(self._mesh.vertices[:, 2], self.zmax)
        ]
        return float(np.max(np.linalg.norm(section[:, :2] - self.centroid[:2], axis=1)))

    @property
    def height(self) -> float:
        return self.zspan

    @property
    def is_truncated(self) -> bool:
        return not np.isclose(self.radius_top, 0)

    @property
    def slant_height(self) -> float:
        return float(np.hypot(self.radius - self.radius_top, self.height))


class Pyramid(Structure):
    """
    Pyramid with a rectangular base in the xy plane and its apex on +z.
    """

    def __init__(
            self,
            width: float,
            depth: float,
            height: float,
            pos: ArrayLike = (0, 0, 0),
    ):
        if min(width, depth, height) <= 0:
            raise ValueError("width, depth and height must all be positive.")

        w, d = width / 2, depth / 2
        vertices = np.array([
            (-w, -d, 0.0), (w, -d, 0.0), (w, d, 0.0), (-w, d, 0.0),
            (0.0, 0.0, height),
        ])
        faces = np.array([
            (0, 2, 1), (0, 3, 2),                       # base
            (0, 1, 4), (1, 2, 4), (2, 3, 4), (3, 0, 4),  # sides
        ])

        mesh = Trimesh(vertices=vertices, faces=faces)
        mesh.fix_normals()
        mesh.apply_translation(-mesh.bounds.mean(axis=0))

        super().__init__(mesh)
        self.translate(pos)

    @classmethod
    def regular(
            cls,
            radius: float,
            sides: int,
            height: float,
            angle_start: float = 90.0,
            pos: ArrayLike = (0, 0, 0),
    ) -> Structure:
        """
        Pyramid on a regular n-gon base of circumscribed radius `radius`. As in
        the 2D module, `angle_start` is the first vertex angle from +x, so the
        default puts a base vertex on +y.
        """
        if sides < 3:
            raise ValueError("A polygon base needs at least three sides.")

        theta = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        theta = theta + np.radians(angle_start)
        base = np.column_stack([
            radius * np.cos(theta), radius * np.sin(theta), np.zeros(sides)
        ])

        vertices = np.vstack([base, [0.0, 0.0, height]])
        faces = [(i, (i + 1) % sides, sides) for i in range(sides)]
        faces += [(0, i, i + 1) for i in range(1, sides - 1)]

        mesh = Trimesh(vertices=vertices, faces=np.array(faces))
        mesh.fix_normals()
        mesh.apply_translation(-mesh.bounds.mean(axis=0))

        structure = Structure(mesh)
        return structure.translate(pos)

    @property
    def width(self) -> float:
        return self.xspan

    @property
    def depth(self) -> float:
        return self.yspan

    @property
    def height(self) -> float:
        return self.zspan

    @property
    def apex(self) -> np.ndarray:
        return self._mesh.vertices[np.argmax(self._mesh.vertices[:, 2])]