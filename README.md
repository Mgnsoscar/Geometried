# geomeTried

Chainable 2D and 3D geometry primitives with a single, consistent API.

The 2D half (`geomeTried`) is built on [shapely](https://shapely.readthedocs.io/)
and the 3D half (`geomeTried3d`) on [trimesh](https://trimesh.org/). Both share
the same conventions: every primitive is centred on the origin and moved by
`pos`, transform methods mutate in place and return `self` so calls chain, and
shape properties such as `radius` or `width` are derived from the geometry so
they stay correct after scaling.

```python
from geomeTried import Rectangle, Circle

plate = (
    Rectangle(10, 4, corner_radius=1)
    .rotate(30)
    .min_to("y", 0)
    .difference(Circle(1, pos=(3, 0)))
)
plate.show()                       # matplotlib preview  (pip install "geometried[plot]")
solid = plate.extrude_polygon(2)   # a 3D Structure      (pip install "geometried[3d]")
```

## Installation

```bash
pip install geometried            # 2D shapes (numpy + shapely)
pip install "geometried[3d]"      # add trimesh-backed 3D solids
pip install "geometried[plot]"    # add matplotlib preview for Shape.show
pip install "geometried[all]"     # everything
```

The distribution is named `geometried`; the import names are **`geomeTried`**
(2D) and **`geomeTried3d`** (3D), matching the original module filenames.

The core install is deliberately light: importing `geomeTried` and using the 2D
`Shape` classes needs only numpy and shapely. The 3D `Structure` classes, and
the `Shape.extrude_polygon` / `sweep_polygon` / `rotate_polygon` constructors
that turn a 2D profile into a solid, require the `[3d]` extra.

## What's included

**2D — `geomeTried`**: `Shape`, `Rectangle`, `Trapezoid`, `Parallelogram`,
`Triangle`, `RightSidedTriangle`, `Circle`, `RegularPolygon` (aliased `NGon`).

**3D — `geomeTried3d`**: `Structure`, `Box`, `Cylinder`, `Sphere`, `Cone`,
`Pyramid`.

**Custom structures — `geomeTried_structures`**: a growing collection of
ready-made parametric solids, each a `Structure` subclass. Currently
`HoneycombCylinder` — a hollow tube whose wall is perforated with radial
hexagonal holes in a seamless honeycomb pattern.

```python
from geomeTried_structures import HoneycombCylinder

part = HoneycombCylinder(
    height=60, outer_radius=30, wall_thickness=4,
    line_thickness=2.5, hole_size=7,   # line_thickness (the web) is exact; holes fill the rest
    orientation="flat_top",            # or "pointy_top"
)
part.export("part.stl")
```

All primitives support translation, rotation, scaling, alignment helpers
(`min_to`, `max_to`, `min_along_to`, …), boolean operations (`union`,
`intersection`, `difference`), convex hulls, and rich extent/anchor properties
(`bbox_center`, corner and face-centre points, projection onto arbitrary axes).

```python
import geomeTried as g2      # 2D
import geomeTried3d as g3    # 3D

hexagon = g2.RegularPolygon.from_side_length(2, 6)
prism   = g3.Box(10, 4, 2).difference(g3.Cylinder(1, 5))
```

## License

MIT — see [LICENSE](LICENSE).
