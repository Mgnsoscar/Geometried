"""
geomeTried_structures.py

A collection of ready-made parametric solids, each a subclass of the trimesh
based `Structure` from geomeTried3d. Import and instantiate them like any other
primitive:

    from geomeTried_structures import HoneycombCylinder

    part = HoneycombCylinder(
        height=60, outer_radius=30, wall_thickness=4,
        line_thickness=2.5, hole_size=7,
    )
    part.export("part.stl")

They follow the same conventions as the built-in primitives: built centred on
the origin, moved by `pos`, transforms mutate in place and return `self`, and
angles are in degrees.
"""

from __future__ import annotations

import numpy as np
import trimesh

import geomeTried as g2  # noqa: F401  (kept for API parity / future structures)
import geomeTried3d as g3
from geomeTried3d import Structure

ArrayLike = np.typing.ArrayLike

__all__ = ["HoneycombCylinder"]

# Pointy-top hexagon corners, counterclockwise, a vertex on top. Multiply by
# the circumradius to get (tangential, axial) offsets for each corner.
_HEX_CORNERS = np.radians(90.0 + 60.0 * np.arange(7))  # 7th closes the loop


def _hex_boundary(a: float, segments: int) -> np.ndarray:
    """
    Boundary of a pointy-top hexagon of circumradius `a` as (tangential, axial)
    offsets, with each of the six edges split into `segments` straight pieces.

    Subdividing the edges matters here: each cutter's side faces are ruled by
    radial generators, so putting several generators along every edge makes the
    hole boundary follow the true straight edge on both the inner and outer
    surface, instead of being kinked by a single quad's diagonal.
    """
    corners = np.column_stack([np.cos(_HEX_CORNERS), np.sin(_HEX_CORNERS)]) * a
    m = np.arange(segments) / segments
    pts = [corners[k] + (corners[k + 1] - corners[k]) * m[:, None] for k in range(6)]
    return np.vstack(pts)   # (6 * segments, 2): columns are tangential, axial


def _cutter_faces(p: int) -> np.ndarray:
    """Faces for one radial cutter: `p` inner-ring verts then `p` outer-ring."""
    side = [[k, (k + 1) % p, p + (k + 1) % p] for k in range(p)] \
        + [[k, p + (k + 1) % p, p + k] for k in range(p)]
    inner_cap = [[0, k, k + 1] for k in range(1, p - 1)]
    outer_cap = [[p, p + k + 1, p + k] for k in range(1, p - 1)]
    return np.array(side + inner_cap + outer_cap, dtype=np.int64)


class HoneycombCylinder(Structure):
    """
    Hollow cylinder whose wall is perforated with hexagonal holes in a
    honeycomb pattern.

    Each hole is a hexagonal wedge focused on the cylinder's centre axis: every
    corner is extruded radially from its own point on the axis, so the hexagon
    keeps its full height through the wall and shrinks only in width toward the
    inner surface (the corners fan out purely tangentially as the radius grows).
    Extrapolated inward, the whole hexagon collapses onto a segment of the axis.

    `line_thickness` and `hole_size` are measured on the outer surface. The web
    is the controlling parameter: the holes are made as large as they can be for
    that web, at a size as close as possible to `hole_size`. Circumferentially
    the pitch is nudged so a whole number of columns closes up (no seam).
    Axially the pattern is centred and clipped by the tube ends, so the top
    cutoff mirrors the bottom.
    """

    # design metadata carried across `copy()` (see below)
    _META = (
        "height", "outer_radius", "wall_thickness", "inner_radius",
        "line_thickness", "hole_size", "hole_across_flats",
        "n_columns", "n_rows", "sections",
    )

    def __init__(
        self,
        height: float,
        outer_radius: float,
        wall_thickness: float,
        line_thickness: float,
        hole_size: float,
        sections: int = 160,
        edge_segments: int = 8,
        clearance: float | None = None,
        pos: ArrayLike = (0, 0, 0),
    ):
        """
        Parameters
        ----------
        height         : overall height of the tube (z extent).
        outer_radius   : outer radius of the wall.
        wall_thickness : radial wall thickness (outer_radius - inner_radius).
        line_thickness : the web between neighbouring holes, measured on the
                         outer surface. The controlling parameter.
        hole_size      : target hexagon size across the flats on the outer
                         surface. Holes are made as large as possible for the
                         given `line_thickness`, as close to this as the
                         wrap-around allows.
        sections       : facet count for the cylinders (higher = rounder).
        edge_segments  : pieces each hexagon edge is split into. Higher values
                         make the tapered hole edges follow the true straight
                         line more closely on the inner surface.
        clearance      : how far each cutter overshoots the wall on each side.
                         Defaults to a quarter of the wall thickness.
        pos            : position of the finished solid's centre.
        """
        if edge_segments < 1:
            raise ValueError("edge_segments must be at least 1.")
        if min(height, outer_radius, wall_thickness, hole_size, line_thickness) <= 0:
            raise ValueError("dimensions must be positive.")
        if wall_thickness >= outer_radius:
            raise ValueError("wall_thickness must be smaller than outer_radius.")

        clearance = 0.25 * wall_thickness if clearance is None else clearance
        inner_radius = outer_radius - wall_thickness

        # --- the plain hollow tube ---------------------------------------- #
        tube = g3.Cylinder(outer_radius, height, sections=sections).difference(
            g3.Cylinder(inner_radius, height + 2.0, sections=sections)
        )

        # --- honeycomb lattice on the OUTER surface ----------------------- #
        # Wrap-neat: choose the column count from the target cell, then
        # back-solve the exact centre pitch. The web is held exact (on the outer
        # surface) and the hole size is whatever is left over.
        r_ref = outer_radius
        n_cols = max(1, round(2.0 * np.pi * r_ref / (hole_size + line_thickness)))
        d = 2.0 * np.pi * r_ref / n_cols                # exact centre pitch
        across_flats = d - line_thickness               # holes fill the rest
        if across_flats <= 0:
            raise ValueError(
                "line_thickness is too large for this hole_size / radius."
            )
        a = across_flats / np.sqrt(3.0)                 # hexagon circumradius

        d_theta = 2.0 * np.pi / n_cols                  # angular column pitch
        row_pitch = d * np.sqrt(3.0) / 2.0              # axial row spacing

        # Fill the full height, centred on z = 0. Rows +j and -j share the same
        # angular offset, so the top and bottom cutoffs match.
        n_rows_half = int(np.floor((height / 2.0 + a) / row_pitch))

        # Each corner is extruded radially from its own point on the axis: the
        # angular offset is fixed while the height z stays constant with radius.
        # So the hexagon keeps its full height and only its width grows with the
        # radius. The cutter spans the wall from just inside to just outside.
        # The hexagon boundary is subdivided so the ruled side faces trace the
        # true straight edges on both surfaces (no diagonal kink).
        r_in = inner_radius - clearance
        r_out = outer_radius + clearance
        boundary = _hex_boundary(a, edge_segments)      # (P, 2): tangential, axial
        p = len(boundary)
        dphi = boundary[:, 0] / r_ref                   # angular offsets (radius-free)
        dz = boundary[:, 1]                             # axial offsets (constant in r)
        faces_template = _cutter_faces(p)

        verts_all, faces_all, base = [], [], 0
        for j in range(-n_rows_half, n_rows_half + 1):
            z0 = j * row_pitch
            theta0 = (0.5 * (j & 1)) * d_theta   # offset odd rows by half a column
            for i in range(n_cols):
                phi = theta0 + i * d_theta + dphi
                cph, sph = np.cos(phi), np.sin(phi)
                z = z0 + dz
                inner = np.column_stack([r_in * cph, r_in * sph, z])
                outer = np.column_stack([r_out * cph, r_out * sph, z])
                verts_all.append(np.vstack([inner, outer]))
                faces_all.append(faces_template + base)
                base += 2 * p

        holes = g3.Structure(_frustum_mesh(
            np.vstack(verts_all), np.vstack(faces_all)
        ))
        result = tube.difference(holes)

        super().__init__(result.mesh)
        self.translate(pos)

        # design metadata (as built, measured on the outer surface)
        self.height = height
        self.outer_radius = outer_radius
        self.wall_thickness = wall_thickness
        self.inner_radius = inner_radius
        self.line_thickness = line_thickness
        self.hole_size = hole_size
        self.hole_across_flats = across_flats
        self.n_columns = n_cols
        self.n_rows = 2 * n_rows_half + 1
        self.sections = sections

    def copy(self):
        new = super().copy()
        for name in self._META:
            setattr(new, name, getattr(self, name))
        return new

    def __repr__(self) -> str:
        return (
            f"HoneycombCylinder(height={self.height:.4g}, "
            f"outer_radius={self.outer_radius:.4g}, "
            f"wall_thickness={self.wall_thickness:.4g}, "
            f"line_thickness={self.line_thickness:.4g}, "
            f"hole_across_flats={self.hole_across_flats:.4g}, "
            f"columns={self.n_columns}, rows={self.n_rows})"
        )


def _frustum_mesh(vertices: np.ndarray, faces: np.ndarray) -> trimesh.Trimesh:
    """Assemble the concatenated cutters into one mesh with outward normals."""
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.fix_normals()
    return mesh
