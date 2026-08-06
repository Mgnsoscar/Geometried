"""
geomeTried2d.py

2D geometry primitives built on shapely, mirroring the API of the trimesh-based
`Structure` classes in base_structure.py.

Conventions
-----------
* Every primitive is built centred on the origin (bounding-box centre) and then
  moved by `pos`.
* Transform methods mutate in place and return `self`, so the concrete subclass
  is preserved and calls chain: ``Rectangle(10, 4).rotate(30).min_to("y", 0)``.
* Boolean operations and `set_width` return a new, generic `Shape`.
* Angles are in degrees.
* Shape-specific properties (`radius`, `width`, `sides`, ...) are derived from
  the geometry rather than stored, so they stay correct after scaling.
"""

from __future__ import annotations

import typing
from typing import Self

import numpy as np
import shapely
import shapely.geometry.polygon
from shapely import affinity
from shapely.geometry.base import BaseGeometry

Axis = typing.Literal["x", "y"]
Extremity = typing.Literal["min", "max"]
ArrayLike = np.typing.ArrayLike
ShapeLike = typing.Union["Shape", BaseGeometry, ArrayLike]

__all__ = [
    "Shape",
    "Rectangle",
    "Trapezoid",
    "Parallelogram",
    "Triangle",
    "RightSidedTriangle",
    "Circle",
    "RegularPolygon",
    "NGon",
]


def _structure_class():
    """
    Import the 3D companion module on demand.

    Keeping this out of the module header means geomeTried2d needs only shapely
    to import; trimesh is required just for the solid constructors below.
    """
    try:
        from .geomeTried3d import Structure
    except ImportError:
        try:
            from geomeTried3d import Structure
        except ImportError as exc:
            raise ImportError(
                "Building a solid from a Shape requires geomeTried3d and "
                f"trimesh; importing it failed with: {exc}"
            ) from exc
    return Structure


class Shape:
    """
    A 2D region backed by a shapely Polygon or MultiPolygon.
    """

    _geom: BaseGeometry

    def __init__(
            self,
            shape: ShapeLike,
            holes: typing.Sequence[ShapeLike] | None = None,
    ) -> None:
        geom = self._coerce(shape)

        if holes:
            cutter = shapely.unary_union([self._coerce(h) for h in holes])
            geom = geom.difference(cutter)

        if geom.is_empty:
            raise ValueError("Resulting geometry is empty.")

        self._geom = geom

    # ------------------------------------------------------------------ #
    # construction helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _coerce(shape: ShapeLike) -> BaseGeometry:
        """Accept a Shape, a shapely geometry, or an (N, 2) vertex array."""
        if isinstance(shape, Shape):
            return shape._geom
        if isinstance(shape, BaseGeometry):
            return shape

        vertices = np.asarray(shape, dtype=float)
        if vertices.ndim != 2 or vertices.shape[1] != 2:
            raise TypeError(
                f"Expected a Shape, a shapely geometry, or an (N, 2) vertex "
                f"array; got an array of shape {vertices.shape}."
            )
        if len(vertices) < 3:
            raise ValueError("At least three vertices are required.")

        return shapely.Polygon(vertices)

    @classmethod
    def from_union(
            cls,
            *shapes: ShapeLike,
            holes: typing.Sequence[ShapeLike] | None = None,
    ) -> Shape:
        """
        Build a single shape from the union of several.

        Takes shapes as separate arguments or as one sequence, and each may be a
        `Shape`, a shapely geometry, or an (N, 2) vertex array:

            Shape.from_union(Rectangle(10, 4), Circle(3, pos=(5, 0)))
            Shape.from_union([Rectangle(10, 4), Circle(3, pos=(5, 0))])

        Always returns a generic `Shape`, never the calling subclass, since the
        union of two rectangles is not itself a rectangle. Inputs that do not
        touch produce a valid multipart shape - see `vertex_rings`.
        """
        if len(shapes) == 1 and isinstance(shapes[0], (list, tuple)):
            candidate = shapes[0]
            if candidate and all(
                isinstance(c, (Shape, BaseGeometry)) for c in candidate
            ):
                shapes = tuple(candidate)

        if not shapes:
            raise ValueError("from_union needs at least one shape.")

        return Shape(
            shapely.unary_union([cls._coerce(s) for s in shapes]), holes=holes
        )

    def copy(self) -> Self:
        new = object.__new__(type(self))
        new._geom = self._geom  # shapely geometries are immutable
        return new

    # ------------------------------------------------------------------ #
    # geometry access
    # ------------------------------------------------------------------ #

    @property
    def geom(self) -> BaseGeometry:
        return self._geom

    @property
    def polygons(self) -> list[shapely.Polygon]:
        if isinstance(self._geom, shapely.MultiPolygon):
            return list(self._geom.geoms)
        return [self._geom]

    @staticmethod
    def _ordered_ring(coords: ArrayLike, centre: ArrayLike) -> np.ndarray:
        """
        Ring vertices, closing point dropped, wound counterclockwise and rolled
        so the first vertex is the one at the smallest angle from the +x axis
        as seen from `centre`.
        """
        ring = np.asarray(coords, dtype=float)[:-1]

        # shoelace: negative area means the ring is wound clockwise
        x, y = ring[:, 0], ring[:, 1]
        if np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)) < 0:
            ring = ring[::-1]

        offset = ring - np.asarray(centre, dtype=float)
        angles = np.arctan2(offset[:, 1], offset[:, 0])

        # map to [0, 2pi), but keep a vertex sitting on the +x axis at 0 rather
        # than letting a tiny negative angle wrap it round to nearly 2pi
        angles[angles < -1e-12] += 2 * np.pi
        angles[angles < 0] = 0.0

        return np.roll(ring, -int(np.argmin(angles)), axis=0)

    @property
    def vertices(self) -> np.ndarray:
        """
        Exterior vertices as an (N, 2) array, counterclockwise, starting from
        the vertex at the smallest angle from the +x axis.

        For a convex shape this is exactly the vertices sorted by angle over
        0..360 degrees. For a concave one the boundary order is kept, since a
        pure angular sort would no longer describe the outline.
        """
        polys = self.polygons
        if len(polys) != 1:
            raise ValueError(
                f"Shape has {len(polys)} disjoint parts, so a single vertex "
                f"ring is ambiguous; use `.vertex_rings` or `.polygons`."
            )
        return self._ordered_ring(polys[0].exterior.coords, self.centroid)

    @property
    def vertex_rings(self) -> list[np.ndarray]:
        """
        Every ring in the shape - each part's exterior followed by its holes -
        ordered the same way as `vertices`.
        """
        rings = []
        for poly in self.polygons:
            centre = np.asarray(poly.centroid.coords[0])
            rings.append(self._ordered_ring(poly.exterior.coords, centre))
            rings.extend(
                self._ordered_ring(r.coords, np.asarray(r.centroid.coords[0]))
                for r in poly.interiors
            )
        return rings

    @property
    def holes(self) -> list[np.ndarray]:
        return [
            self._ordered_ring(ring.coords, np.asarray(ring.centroid.coords[0]))
            for poly in self.polygons
            for ring in poly.interiors
        ]

    @property
    def _points(self) -> np.ndarray:
        rings = []
        for poly in self.polygons:
            rings.append(np.asarray(poly.exterior.coords))
            rings.extend(np.asarray(r.coords) for r in poly.interiors)
        return np.vstack(rings)

    @property
    def is_valid(self) -> bool:
        return self._geom.is_valid

    @property
    def area(self) -> float:
        return self._geom.area

    @property
    def perimeter(self) -> float:
        return self._geom.length

    # ------------------------------------------------------------------ #
    # position and extent
    # ------------------------------------------------------------------ #

    def _get_extremity(self, axis: Axis, type: Extremity = "min") -> float:
        xmin, ymin, xmax, ymax = self._geom.bounds
        return {
            ("x", "min"): xmin,
            ("x", "max"): xmax,
            ("y", "min"): ymin,
            ("y", "max"): ymax,
        }[(axis, type)]

    def _get_span(self, axis: Axis) -> float:
        return getattr(self, f"{axis}max") - getattr(self, f"{axis}min")

    @property
    def centroid(self) -> np.ndarray:
        return np.asarray(self._geom.centroid.coords[0])

    @property
    def x(self) -> float:
        return self.centroid[0]

    @property
    def y(self) -> float:
        return self.centroid[1]

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
    def xspan(self) -> float:
        return self._get_span("x")

    @property
    def yspan(self) -> float:
        return self._get_span("y")

    @property
    def bbox_center(self) -> np.ndarray:
        xmin, ymin, xmax, ymax = self._geom.bounds
        return np.array([(xmin + xmax) / 2, (ymin + ymax) / 2])

    # -- bounding box corners ------------------------------------------- #

    @property
    def xminymin(self) -> np.ndarray:
        return np.array([self.xmin, self.ymin])

    @property
    def xminymax(self) -> np.ndarray:
        return np.array([self.xmin, self.ymax])

    @property
    def xmaxymin(self) -> np.ndarray:
        return np.array([self.xmax, self.ymin])

    @property
    def xmaxymax(self) -> np.ndarray:
        return np.array([self.xmax, self.ymax])

    # -- bounding box edge midpoints ------------------------------------ #

    @property
    def xmincenter(self) -> np.ndarray:
        return np.array([self.xmin, self.bbox_center[1]])

    @property
    def xmaxcenter(self) -> np.ndarray:
        return np.array([self.xmax, self.bbox_center[1]])

    @property
    def ymincenter(self) -> np.ndarray:
        return np.array([self.bbox_center[0], self.ymin])

    @property
    def ymaxcenter(self) -> np.ndarray:
        return np.array([self.bbox_center[0], self.ymax])

    # ------------------------------------------------------------------ #
    # projection onto an arbitrary axis
    # ------------------------------------------------------------------ #

    @staticmethod
    def _unit(vector: ArrayLike) -> np.ndarray:
        v = np.asarray(vector, dtype=float).ravel()[:2]
        norm = np.linalg.norm(v)
        if np.isclose(norm, 0):
            raise ValueError("Axis vector has zero length.")
        return v / norm

    def min_along(self, vector: ArrayLike) -> float:
        """Smallest coordinate of the shape projected onto `vector`."""
        return float(np.min(self._points @ self._unit(vector)))

    def max_along(self, vector: ArrayLike) -> float:
        """Largest coordinate of the shape projected onto `vector`."""
        return float(np.max(self._points @ self._unit(vector)))

    def span_along(self, vector: ArrayLike) -> float:
        projection = self._points @ self._unit(vector)
        return float(np.max(projection) - np.min(projection))

    # ------------------------------------------------------------------ #
    # transforms - return the same type
    # ------------------------------------------------------------------ #

    def translate(self, vector: ArrayLike) -> Self:
        dx, dy = np.asarray(vector, dtype=float).ravel()[:2]
        self._geom = affinity.translate(self._geom, dx, dy)
        return self

    def set_position(self, pos: ArrayLike) -> Self:
        return self.translate(np.asarray(pos, dtype=float).ravel()[:2] - self.centroid)

    def rotate(self, angle: float, point: ArrayLike | None = None) -> Self:
        """
        Rotate by `angle` degrees, about the centroid unless `point` is given.
        """
        origin = "centroid" if point is None else tuple(
            np.asarray(point, dtype=float).ravel()[:2]
        )
        self._geom = affinity.rotate(self._geom, angle, origin=origin)
        return self

    def scale(
            self,
            factor: float,
            factor_y: float | None = None,
            point: ArrayLike | None = None,
    ) -> Self:
        """
        Scale about the centroid unless `point` is given. Pass `factor_y` for a
        non-uniform scale.
        """
        origin = "centroid" if point is None else tuple(
            np.asarray(point, dtype=float).ravel()[:2]
        )
        self._geom = affinity.scale(
            self._geom, factor, factor if factor_y is None else factor_y, origin=origin
        )
        return self

    def min_to(self, axis: Axis, pos: float, buffer: float = 0.0) -> Self:
        delta = (pos - getattr(self, f"{axis}min")) + buffer
        return self.translate((delta, 0) if axis == "x" else (0, delta))

    def max_to(self, axis: Axis, pos: float, buffer: float = 0.0) -> Self:
        delta = (pos - getattr(self, f"{axis}max")) + buffer
        return self.translate((delta, 0) if axis == "x" else (0, delta))

    def min_along_to(self, axis: ArrayLike, pos: float, buffer: float = 0.0) -> Self:
        unit = self._unit(axis)
        return self.translate(unit * ((pos - self.min_along(unit)) + buffer))

    def max_along_to(self, axis: ArrayLike, pos: float, buffer: float = 0.0) -> Self:
        unit = self._unit(axis)
        return self.translate(unit * ((pos - self.max_along(unit)) + buffer))

    # ------------------------------------------------------------------ #
    # operations - return a generic Shape
    # ------------------------------------------------------------------ #

    def union(self, *shapes: ShapeLike) -> Shape:
        return Shape(shapely.unary_union(
            [self._geom, *(self._coerce(s) for s in shapes)]
        ))

    def intersection(self, *shapes: ShapeLike) -> Shape:
        geom = self._geom
        for shape in shapes:
            geom = geom.intersection(self._coerce(shape))
        return Shape(geom)

    def difference(self, *shapes: ShapeLike) -> Shape:
        if not shapes:
            return Shape(self._geom)
        cutter = shapely.unary_union([self._coerce(s) for s in shapes])
        return Shape(self._geom.difference(cutter))

    def set_width(self, axis: Axis, width: float, point: float | None = None) -> Shape:
        """
        Scale along `axis` so that the span becomes `width`, holding the line at
        `point` fixed. Defaults to the centroid coordinate on that axis.
        """
        span = getattr(self, f"{axis}span")
        if np.isclose(span, 0):
            raise ValueError(
                f"Cannot set width along {axis!r}: current {axis}span is zero."
            )

        if point is None:
            point = getattr(self, axis)

        factor = width / span
        origin = (point, 0.0) if axis == "x" else (0.0, point)
        xfact, yfact = (factor, 1.0) if axis == "x" else (1.0, factor)

        return Shape(affinity.scale(self._geom, xfact, yfact, origin=origin))

    def offset(self, distance: float, **kwargs) -> Shape:
        """Outward (or inward, if negative) offset of the boundary."""
        return Shape(self._geom.buffer(distance, **kwargs))

    def convex_hull(self, *shapes: ShapeLike) -> Shape:
        return Shape(self.union(*shapes)._geom.convex_hull)

    # ------------------------------------------------------------------ #
    # solid constructors - return a geomeTried3d Structure
    # ------------------------------------------------------------------ #

    def extrude_polygon(self, height: float, centred: bool = True, **kwargs):
        """
        Extrude this shape along +z into a 3D `Structure`. Holes carry through.

        The shape keeps its x/y position. With `centred` the solid straddles
        z = 0; otherwise it runs from z = 0 to z = height.
        """
        return _structure_class().extrude_polygon(
            self, height, centred=centred, **kwargs
        )

    def sweep_polygon(
            self,
            path: ArrayLike,
            angles: ArrayLike | None = None,
            cap: bool = True,
            **kwargs,
    ):
        """
        Sweep this shape along a 3D path, given as an (N, 3) array of points,
        into a `Structure`. `angles` optionally twists the profile, in radians.
        """
        return _structure_class().sweep_polygon(
            self, path, angles=angles, cap=cap, **kwargs
        )

    def rotate_polygon(
            self,
            angle: float = 360.0,
            sections: int = 64,
            cap: bool | None = None,
            **kwargs,
    ):
        """
        Revolve this shape about the z axis into a solid of revolution.

        The shape is read as a lathe profile: its x is radius and its y becomes
        height, so it must not cross into negative x. `angle` is the sweep in
        degrees. Not to be confused with `rotate`, which turns the shape within
        the 2D plane.

            Rectangle(2, 5).translate((6, 0)).rotate_polygon()   # a square torus
        """
        return _structure_class().rotate_polygon(
            self, angle=angle, sections=sections, cap=cap, **kwargs
        )

    # ------------------------------------------------------------------ #
    # queries and display
    # ------------------------------------------------------------------ #

    def intersects_with(self, *shapes: ShapeLike) -> bool:
        return any(self._geom.intersects(self._coerce(s)) for s in shapes)

    def contains(self, other: ShapeLike) -> bool:
        return self._geom.contains(self._coerce(other))

    def _path(self):
        """Compound matplotlib Path, with interior rings as real holes."""
        from matplotlib.path import Path

        vertices, codes = [], []
        for poly in self.polygons:
            # exterior CCW, interiors CW, so nonzero winding cuts the holes
            poly = shapely.geometry.polygon.orient(poly, sign=1.0)
            for ring in (poly.exterior, *poly.interiors):
                coords = np.asarray(ring.coords)
                vertices.append(coords)
                codes.append(
                    [Path.MOVETO]
                    + [Path.LINETO] * (len(coords) - 2)
                    + [Path.CLOSEPOLY]
                )

        return Path(np.vstack(vertices), np.concatenate(codes))

    def show(self, *shapes: Shape, ax=None, block: bool = True, **kwargs):
        """
        Quick matplotlib preview. Returns the axes.

        Pass `ax` to draw into an existing plot, or `block=False` to keep the
        call non-blocking and manage `plt.show()` yourself.
        """
        import matplotlib.pyplot as plt
        from matplotlib.patches import PathPatch

        created = ax is None
        if created:
            _, ax = plt.subplots()

        style = {"alpha": 0.4, "edgecolor": "black", "linewidth": 1.0}
        style.update(kwargs)
        facecolor = style.pop("facecolor", None)
        colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", ["C0"])

        for i, shape in enumerate((self, *shapes)):
            ax.add_patch(PathPatch(
                shape._path(),
                facecolor=facecolor or colors[i % len(colors)],
                **style,
            ))

        ax.set_aspect("equal")
        ax.relim()
        ax.autoscale_view()

        if created:
            plt.show(block=block)

        return ax

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(x={self.x:.4g}, y={self.y:.4g}, "
            f"xspan={self.xspan:.4g}, yspan={self.yspan:.4g})"
        )


# ====================================================================== #
# primitives
# ====================================================================== #


class Rectangle(Shape):

    def __init__(self, width: float, height: float, pos: ArrayLike = (0, 0), corner_radius = 0):
        w, h = width / 2, height / 2
        rect = Shape([(-w, -h), (w, -h), (w, h), (-w, h)])

        if corner_radius != 0:
            corner_circle = Circle(corner_radius, sections=500)
            corners = []
            for corner in rect.vertices:
                corner = (
                    corner_circle.copy()
                    .set_position(corner + np.asarray((corner_radius, corner_radius)) * np.sign(corner) * -1)
                )
                corners.append(corner)
            rect = Shape.from_union(*corners).convex_hull()

        super().__init__(rect.vertices)

        self.translate(pos)

    @classmethod
    def from_bounds(cls, xmin: float, ymin: float, xmax: float, ymax: float) -> Self:
        return cls(xmax - xmin, ymax - ymin, pos=((xmin + xmax) / 2, (ymin + ymax) / 2))

    @classmethod
    def square(cls, side: float, pos: ArrayLike = (0, 0)) -> Self:
        return cls(side, side, pos=pos)

    @property
    def width(self) -> float:
        return self.xspan

    @property
    def height(self) -> float:
        return self.yspan

    @property
    def diagonal(self) -> float:
        return float(np.hypot(self.width, self.height))


class Trapezoid(Shape):
    """
    Isosceles by default; `offset` shifts the top edge along x.
    """

    def __init__(
            self,
            width_bottom: float,
            width_top: float,
            height: float,
            offset: float = 0.0,
            pos: ArrayLike = (0, 0),
    ):
        b, t, h = width_bottom / 2, width_top / 2, height / 2
        super().__init__([
            (-b, -h), (b, -h), (t + offset, h), (-t + offset, h),
        ])
        self.translate(np.asarray(pos, dtype=float) - self.bbox_center)

    def _edge_width(self, at_max: bool) -> float:
        v = self.vertices
        edge = v[np.isclose(v[:, 1], self.ymax if at_max else self.ymin)]
        return float(np.ptp(edge[:, 0]))

    @property
    def width_bottom(self) -> float:
        return self._edge_width(at_max=False)

    @property
    def width_top(self) -> float:
        return self._edge_width(at_max=True)

    @property
    def height(self) -> float:
        return self.yspan


class Parallelogram(Shape):
    """
    `width` is the length of the base edge; `angle` is the interior angle at the
    base in degrees. The bounding box is wider than `width` unless angle == 90.
    """

    def __init__(
            self,
            width: float,
            height: float,
            angle: float = 90.0,
            pos: ArrayLike = (0, 0),
    ):
        if np.isclose(angle % 180, 0):
            raise ValueError("angle must not be a multiple of 180 degrees.")

        skew = height / np.tan(np.radians(angle))
        w, h = width / 2, height / 2
        super().__init__([
            (-w, -h), (w, -h), (w + skew, h), (-w + skew, h),
        ])
        self.translate(np.asarray(pos, dtype=float) - self.bbox_center)

    @classmethod
    def from_skew(
            cls,
            width: float,
            height: float,
            skew: float,
            pos: ArrayLike = (0, 0),
    ) -> Self:
        angle = np.degrees(np.arctan2(height, skew))
        return cls(width, height, angle=angle, pos=pos)

    @property
    def _edges(self) -> tuple[np.ndarray, np.ndarray]:
        v = self.vertices
        return (
            v[np.isclose(v[:, 1], self.ymin)][:, 0],
            v[np.isclose(v[:, 1], self.ymax)][:, 0],
        )

    @property
    def width(self) -> float:
        return float(np.ptp(self._edges[0]))

    @property
    def height(self) -> float:
        return self.yspan

    @property
    def skew(self) -> float:
        bottom, top = self._edges
        return float(top.mean() - bottom.mean())

    @property
    def angle(self) -> float:
        return float(np.degrees(np.arctan2(self.height, self.skew)) % 180)


class Triangle(Shape):
    """
    An arbitrary triangle. The regular constructor takes three vertices; the
    classmethods build one from side/angle combinations.
    """

    def __init__(self, *points: ArrayLike):
        """
        Build a triangle from three vertices, either as three separate points
        or as a single (3, 2) array. The vertices are used as given - unlike the
        classmethods, no recentring is applied.
        """
        vertices = np.asarray(points[0] if len(points) == 1 else points, dtype=float)

        if vertices.shape != (3, 2):
            raise ValueError(
                f"Expected three (x, y) points or a single (3, 2) array; "
                f"got an array of shape {vertices.shape}."
            )
        if np.isclose(np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0]), 0):
            raise ValueError("The three points are collinear.")

        super().__init__(vertices)

    @classmethod
    def _centred(cls, vertices: ArrayLike, pos: ArrayLike) -> Self:
        triangle = cls(*np.asarray(vertices, dtype=float))
        return triangle.translate(np.asarray(pos, dtype=float) - triangle.bbox_center)

    @classmethod
    def from_sides(cls, a: float, b: float, c: float, pos: ArrayLike = (0, 0)) -> Self:
        """SSS. `c` is the base; `a` is opposite vertex A, `b` opposite B."""
        sides = sorted((a, b, c))
        if sides[0] <= 0 or sides[0] + sides[1] <= sides[2]:
            raise ValueError(f"Sides {a}, {b}, {c} do not satisfy the triangle inequality.")

        x = (b ** 2 + c ** 2 - a ** 2) / (2 * c)
        y = np.sqrt(max(b ** 2 - x ** 2, 0.0))
        return cls._centred([(0, 0), (c, 0), (x, y)], pos)

    @classmethod
    def from_side_angle_side(
            cls, side_1: float, angle: float, side_2: float, pos: ArrayLike = (0, 0)
    ) -> Self:
        """SAS. `angle` in degrees is enclosed between the two sides."""
        if not 0 < angle % 360 < 180:
            raise ValueError("Enclosed angle must be strictly between 0 and 180 degrees.")
        theta = np.radians(angle)
        return cls._centred(
            [(0, 0), (side_1, 0), (side_2 * np.cos(theta), side_2 * np.sin(theta))], pos
        )

    @classmethod
    def from_angle_side_angle(
            cls, angle_1: float, side: float, angle_2: float, pos: ArrayLike = (0, 0)
    ) -> Self:
        """ASA. `side` lies between the two given angles, in degrees."""
        if angle_1 <= 0 or angle_2 <= 0 or angle_1 + angle_2 >= 180:
            raise ValueError("Angles must be positive and sum to less than 180 degrees.")

        a1, a2 = np.radians(angle_1), np.radians(angle_2)
        length = side * np.sin(a2) / np.sin(np.pi - a1 - a2)
        return cls._centred(
            [(0, 0), (side, 0), (length * np.cos(a1), length * np.sin(a1))], pos
        )

    @classmethod
    def from_base_and_height(
            cls,
            base: float,
            height: float,
            apex_offset: float = 0.0,
            pos: ArrayLike = (0, 0),
    ) -> Self:
        """Isosceles unless `apex_offset` shifts the apex along the base."""
        return cls._centred(
            [(0, 0), (base, 0), (base / 2 + apex_offset, height)], pos
        )

    @classmethod
    def equilateral(cls, side: float, pos: ArrayLike = (0, 0)) -> Self:
        return cls.from_sides(side, side, side, pos=pos)

    @property
    def sides(self) -> np.ndarray:
        """Side lengths, ordered opposite to `vertices`."""
        v = self.vertices
        return np.array([
            np.linalg.norm(v[2] - v[1]),
            np.linalg.norm(v[2] - v[0]),
            np.linalg.norm(v[1] - v[0]),
        ])

    @property
    def angles(self) -> np.ndarray:
        """Interior angles in degrees, at each vertex in `vertices` order."""
        a, b, c = self.sides
        return np.degrees([
            np.arccos(np.clip((b ** 2 + c ** 2 - a ** 2) / (2 * b * c), -1, 1)),
            np.arccos(np.clip((a ** 2 + c ** 2 - b ** 2) / (2 * a * c), -1, 1)),
            np.arccos(np.clip((a ** 2 + b ** 2 - c ** 2) / (2 * a * b), -1, 1)),
        ])


class RightSidedTriangle(Triangle):
    """
    Right angle at the lower-left corner of the bounding box.
    """

    def __init__(self, width: float, height: float, pos: ArrayLike = (0, 0)):
        super().__init__((0, 0), (width, 0), (0, height))
        self.translate(np.asarray(pos, dtype=float) - self.bbox_center)

    @classmethod
    def from_leg_and_angle(
            cls, leg: float, angle: float, pos: ArrayLike = (0, 0)
    ) -> Self:
        """`angle` in degrees is measured at the far end of `leg`."""
        if not 0 < angle < 90:
            raise ValueError("angle must be strictly between 0 and 90 degrees.")
        return cls(leg, leg * np.tan(np.radians(angle)), pos=pos)

    @classmethod
    def from_hypotenuse_and_angle(
            cls, hypotenuse: float, angle: float, pos: ArrayLike = (0, 0)
    ) -> Self:
        if not 0 < angle < 90:
            raise ValueError("angle must be strictly between 0 and 90 degrees.")
        theta = np.radians(angle)
        return cls(hypotenuse * np.cos(theta), hypotenuse * np.sin(theta), pos=pos)

    @property
    def width(self) -> float:
        return self.xspan

    @property
    def height(self) -> float:
        return self.yspan

    @property
    def hypotenuse(self) -> float:
        return float(np.hypot(self.width, self.height))


class Circle(Shape):
    """
    A circle, or an ellipse when `radius_minor` is given. `radius` is the
    semi-axis along x, `radius_minor` the semi-axis along y.
    """

    def __init__(
            self,
            radius: float,
            radius_minor: float | None = None,
            sections: int = 64,
            pos: ArrayLike = (0, 0),
    ):
        if radius <= 0:
            raise ValueError("radius must be positive.")
        if sections < 3:
            raise ValueError("sections must be at least 3.")

        rx = radius
        ry = radius if radius_minor is None else radius_minor
        if ry <= 0:
            raise ValueError("radius_minor must be positive.")

        theta = np.linspace(0, 2 * np.pi, sections, endpoint=False)
        super().__init__(np.column_stack([rx * np.cos(theta), ry * np.sin(theta)]))
        self.translate(pos)

    @classmethod
    def from_diameter(
            cls, diameter: float, diameter_minor: float | None = None, **kwargs
    ) -> Self:
        return cls(
            diameter / 2,
            None if diameter_minor is None else diameter_minor / 2,
            **kwargs,
        )

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
                "Shape is an ellipse; use `radius_x` / `radius_y` instead."
            )
        return self.radius_x

    @property
    def is_ellipse(self) -> bool:
        return not np.isclose(self.radius_x, self.radius_y)

    @property
    def circumference(self) -> float:
        """Analytic value - Ramanujan's approximation for the elliptical case."""
        a, b = self.radius_x, self.radius_y
        if np.isclose(a, b):
            return float(2 * np.pi * a)
        h = ((a - b) ** 2) / ((a + b) ** 2)
        return float(np.pi * (a + b) * (1 + (3 * h) / (10 + np.sqrt(4 - 3 * h))))


class RegularPolygon(Shape):
    """
    Regular n-gon defined by its circumscribed (outer) radius.

    `angle_start` is the position of the first vertex in degrees, measured from
    the +x axis. It defaults to 90, which points that vertex straight up.
    """

    def __init__(
            self,
            radius: float,
            sides: int,
            angle_start: float = 90.0,
            pos: ArrayLike = (0, 0),
    ):
        if sides < 3:
            raise ValueError("A polygon needs at least three sides.")
        if radius <= 0:
            raise ValueError("radius must be positive.")

        theta = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        theta = theta + np.radians(angle_start)
        super().__init__(
            np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])
        )
        self.translate(pos)

    @classmethod
    def from_inradius(cls, inradius: float, sides: int, **kwargs) -> Self:
        """`inradius` is the apothem - centre to edge midpoint."""
        return cls(inradius / np.cos(np.pi / sides), sides, **kwargs)

    @classmethod
    def from_width_across_flats(cls, width: float, sides: int, **kwargs) -> Self:
        """Spanner size: distance between opposite edges (even `sides`)."""
        return cls.from_inradius(width / 2, sides, **kwargs)

    @classmethod
    def from_side_length(cls, side_length: float, sides: int, **kwargs) -> Self:
        return cls(side_length / (2 * np.sin(np.pi / sides)), sides, **kwargs)

    @classmethod
    def from_area(cls, area: float, sides: int, **kwargs) -> Self:
        radius = np.sqrt(2 * area / (sides * np.sin(2 * np.pi / sides)))
        return cls(radius, sides, **kwargs)

    @property
    def sides(self) -> int:
        return len(self.vertices)

    @property
    def radius(self) -> float:
        """Circumradius, measured from the centroid."""
        return float(np.max(np.linalg.norm(self.vertices - self.centroid, axis=1)))

    @property
    def inradius(self) -> float:
        return float(self.radius * np.cos(np.pi / self.sides))

    @property
    def side_length(self) -> float:
        return float(2 * self.radius * np.sin(np.pi / self.sides))

    @property
    def width_across_flats(self) -> float:
        return 2 * self.inradius


NGon = RegularPolygon