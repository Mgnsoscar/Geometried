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

import geomeTried as g2
import geomeTried3d as g3
from geomeTried3d import Structure

ArrayLike = np.typing.ArrayLike

__all__ = ["HoneycombCylinder"]


class HoneycombCylinder(Structure):
    """
    Hollow cylinder whose wall is perforated with hexagonal holes in a
    honeycomb pattern.

    Each hole is a hexagonal prism whose long axis is radial - it points
    straight at the cylinder's centre axis and punches through the wall.

    The line thickness between holes (the web) is the controlling parameter: it
    is held uniform and exact, and the holes are made as large as they can be
    for that web, at a size as close as possible to `hole_size`.
    Circumferentially the pitch is nudged so a whole number of columns closes up
    (no seam). Axially the pattern is centred and clipped by the tube ends, so
    the top cutoff mirrors the bottom.
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
        line_thickness : the web between neighbouring holes. Held uniform and
                         exact in every direction - the controlling parameter.
        hole_size      : target hexagon size across the flats. Holes are made as
                         large as possible for the given `line_thickness`, as
                         close to this target as the wrap-around allows.
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
        r_mid = outer_radius - wall_thickness / 2.0

        # --- the plain hollow tube ---------------------------------------- #
        tube = g3.Cylinder(outer_radius, height, sections=sections).difference(
            g3.Cylinder(inner_radius, height + 2.0, sections=sections)
        )

        # --- honeycomb lattice on the mid-wall surface -------------------- #
        # Wrap-neat: choose the column count from the target cell, then
        # back-solve the exact centre pitch. The web is held exact and the hole
        # size is whatever is left over (as big as possible for that web).
        n_cols = max(1, round(2.0 * np.pi * r_mid / (hole_size + line_thickness)))
        d = 2.0 * np.pi * r_mid / n_cols                # exact centre pitch
        across_flats = d - line_thickness               # holes fill the rest
        if across_flats <= 0:
            raise ValueError(
                "line_thickness is too large for this hole_size / radius."
            )
        a = across_flats / np.sqrt(3.0)                 # hexagon circumradius

        d_theta = 2.0 * np.pi / n_cols                  # angular column pitch
        row_pitch = d * np.sqrt(3.0) / 2.0              # axial row spacing

        # Fill the full height, centred on z = 0: keep every row whose hexagon
        # reaches into the tube. Rows +j and -j share the same angular offset,
        # so the top and bottom cutoffs match.
        n_rows_half = int(np.floor((height / 2.0 + a) / row_pitch))

        # --- one radial hex-prism cutter, oriented for theta = 0 ---------- #
        # Pointy-top hexagon in the x-y plane: across-flats along x, a vertex on
        # +y. Extrude along z, then swing z -> radial and y -> world-z so the
        # hexagon's points run along the cylinder axis everywhere.
        length = wall_thickness + 2.0 * clearance
        base = (
            g2.RegularPolygon.from_width_across_flats(across_flats, 6)
            .extrude_polygon(length)    # prism axis +z
        )
        base.rotate(90, axis=(1, 0, 0))  # hexagon points -> world +z
        base.rotate(90, axis=(0, 0, 1))  # prism axis -> +x (radial at theta = 0)

        cutters = []
        for j in range(-n_rows_half, n_rows_half + 1):
            z0 = j * row_pitch
            theta0 = (0.5 * (j & 1)) * d_theta   # offset odd rows by half a column
            for i in range(n_cols):
                theta = theta0 + i * d_theta
                c = base.copy()
                c.rotate(np.degrees(theta), axis=(0, 0, 1), point=(0, 0, 0))
                c.translate((r_mid * np.cos(theta), r_mid * np.sin(theta), z0))
                cutters.append(c.mesh)

        # Disjoint cutters: concatenate into one mesh (no boolean union needed),
        # then subtract in a single pass.
        holes = g3.Structure(trimesh.util.concatenate(cutters))
        result = tube.difference(holes)

        super().__init__(result.mesh)
        self.translate(pos)

        # design metadata (as built)
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
