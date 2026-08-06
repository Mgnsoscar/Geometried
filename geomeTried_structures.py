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

# Pointy-top hexagon, vertices counterclockwise, a vertex on top. Multiply by
# the circumradius to get (tangential, axial) offsets for each corner.
_HEX_ANGLE = np.radians(90.0 + 60.0 * np.arange(6))


class HoneycombCylinder(Structure):
    """
    Hollow cylinder whose wall is perforated with hexagonal holes in a
    honeycomb pattern.

    Each hole is a hexagonal wedge focused on the cylinder's centre axis: its
    six corners ride rays from a single point on the axis, so the hexagon
    scales uniformly with radius and stays a proper regular hexagon everywhere
    through the wall - a little smaller on the inner surface, a little larger on
    the outer. Extrapolated inward, all six corners meet at that axis point.

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
        clearance      : how far each cutter overshoots the wall on each side.
                         Defaults to a quarter of the wall thickness.
        pos            : position of the finished solid's centre.
        """
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

        # Each corner rides a ray from a point on the axis at the hole's height:
        # the angular offset is fixed and the axial offset scales with radius,
        # exactly as the tangential extent does. The hexagon therefore scales
        # uniformly with radius and stays regular through the wall. The cutter
        # spans the wall from just inside to just outside; extended inward all
        # six corners meet the axis point.
        r_in = inner_radius - clearance
        r_out = outer_radius + clearance
        t = a * np.cos(_HEX_ANGLE)          # tangential corner offsets (@ r_ref)
        dz = a * np.sin(_HEX_ANGLE)         # axial corner offsets (@ r_ref)
        dphi = t / r_ref                    # angular corner offsets (radius-free)

        verts_all, faces_all, base = [], [], 0
        for j in range(-n_rows_half, n_rows_half + 1):
            z0 = j * row_pitch
            theta0 = (0.5 * (j & 1)) * d_theta   # offset odd rows by half a column
            for i in range(n_cols):
                phi = theta0 + i * d_theta + dphi
                cph, sph = np.cos(phi), np.sin(phi)
                inner = np.column_stack(
                    [r_in * cph, r_in * sph, z0 + (r_in / r_ref) * dz]
                )
                outer = np.column_stack(
                    [r_out * cph, r_out * sph, z0 + (r_out / r_ref) * dz]
                )
                verts_all.append(np.vstack([inner, outer]))
                faces_all.append(_FRUSTUM_FACES + base)
                base += 12

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


# Faces of one 12-vertex hexagonal frustum: 0..5 inner ring, 6..11 outer ring.
# Six side quads (two triangles each) plus a triangulated cap on each ring.
_FRUSTUM_FACES = np.array(
    [[k, (k + 1) % 6, 6 + (k + 1) % 6] for k in range(6)]
    + [[k, 6 + (k + 1) % 6, 6 + k] for k in range(6)]
    + [[0, k, k + 1] for k in range(1, 5)]              # inner cap
    + [[6, 6 + k + 1, 6 + k] for k in range(1, 5)],     # outer cap
    dtype=np.int64,
)


def _frustum_mesh(vertices: np.ndarray, faces: np.ndarray) -> trimesh.Trimesh:
    """Assemble the concatenated cutters into one mesh with outward normals."""
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.fix_normals()
    return mesh
