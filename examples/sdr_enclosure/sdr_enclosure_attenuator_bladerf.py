"""
sdr_enclosure_attenuator_bladerf.py

The two-storey Pi 5 + attenuator enclosure of sdr_enclosure_attenuator.py,
with a Nuand bladeRF 2.0 micro (xA4 / xA9, board only) in place of the
B206mini-i. Nothing in the other scripts changes; this file only swaps the
board parameters and the preview / drawing labels.

    python examples/sdr_enclosure/sdr_enclosure_attenuator_bladerf.py [output_dir]

bladeRF 2.0 micro mechanical data (from Nuand's micro.dxf and estimate.STEP;
board origin at the corner nearest the DC jack, x along the long edge):
  PCB 101.6 x 63.5 x 1.7 mm. Mounting holes Ø2.54 at (6.35, 6.35),
  (88.9, 6.35), (6.35, 57.15), (88.9, 57.15). SMA edge jacks on the +x short
  edge, 9.35 mm protrusion, centred at y = 54.05 (TX1), 38.8 (RX1),
  24.15 (RX2), 8.95 (TX2). USB 3.0 Type-B on the -x edge at y = 32 (5 mm
  protrusion), DC barrel jack on the same edge at y = 15.35. Tallest part
  13 mm above the PCB, 1.85 mm below.

Layout: the bladeRF lies beside the Pi with its USB/DC edge facing the cable
bay and its SMAs pointing into the +X channel. TX1 sits nearest the back
wall, right above the attenuator's internal SMA, and is the port the jumper
goes to. The straight USB 3.0 Type-B plug needs the same 45 mm bay as the
attenuator's USB-B.
"""

from __future__ import annotations

import os
import sys

import trimesh

sys.path.insert(0, os.path.dirname(__file__))

import sdr_enclosure_attenuator as A  # noqa: E402
from sdr_enclosure import box, cyl_x  # noqa: E402

BLADERF = dict(
    sdr_len=101.6, sdr_wid=63.5, sdr_pcb=1.7,
    sdr_hole_d=2.54,
    sdr_holes=[(6.35, 6.35), (88.9, 6.35), (6.35, 57.15), (88.9, 57.15)],   # (x from USB edge, y from back edge)
    sdr_sma_y=(9.45, 24.7, 39.35, 54.55),   # TX1, RX1, RX2, TX2 - from the back (+Y) long edge
    sdr_sma_labels=("TX1", "RX1", "RX2", "TX2"),
    sdr_sma_len=9.35,
    sdr_usbc_y=31.5,            # USB 3.0 Type-B centre, from the back edge
    sdr_dc_y=48.15,             # DC barrel jack centre, from the back edge
    sdr_top=13.0, sdr_under=1.85,
    sdr_standoff=5.0,
    standoff_sdr_d=6.5, standoff_sdr_hole=2.2,   # M2.5 thread-forming (holes are Ø2.54)
    bay_len=45.0,               # straight USB 3.0 Type-B plug + bend
)

# The attenuator module reads its PARAMS at call time, so overriding them here
# re-targets every part builder without copying the geometry code.
A.PARAMS.update(BLADERF)
PARAMS = A.PARAMS


def layout() -> dict:
    L = A.layout()
    L["sdr_holes"] = list(PARAMS["sdr_holes"])     # explicit, not centred-pitch
    return L


def placeholders(L: dict) -> dict:
    p = PARAMS
    out = A.placeholders(L)
    sdr = L["sdr"]
    uy = L["sdr_map"](0, p["sdr_usbc_y"])[1]
    dy = L["sdr_map"](0, p["sdr_dc_y"])[1]
    # receptacles overhanging the edge, and a straight USB 3.0 Type-B plug
    usb_recep = box(sdr["x0"] - 5.0, sdr["x0"] + 13.1, uy - 6.9, uy + 6.9, sdr["z_pcb1"], sdr["z_pcb1"] + 11.3)
    dc_jack = box(sdr["x0"] - 3.8, sdr["x0"] + 12.5, dy - 5.4, dy + 5.4, sdr["z_pcb1"], sdr["z_pcb1"] + 11.0)
    out["sdr"] = out["sdr"].union(usb_recep, dc_jack).set_color(50, 100, 190)
    if p["plugs"] == "straight":
        usb_b_plug = box(sdr["x0"] - 5.0 - 34.0, sdr["x0"] - 5.0, uy - 6.0, uy + 6.0,
                         sdr["z_pcb1"] + 0.5, sdr["z_pcb1"] + 11.0)
        out["plugs"] = out["plugs"].union(usb_b_plug).set_color(230, 140, 40)
    return out


def plan_drawing(L: dict, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    p = PARAMS
    w = p["wall"]
    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5))
    for ax, title in zip(axes, ("Lower box (attenuator)", "Tray (Pi 5 + bladeRF 2.0 micro)")):
        ax.add_patch(Rectangle((-w, -w), L["x_out"], L["y_out"], fc="#e6e6e6", ec="k", lw=1.5))
        ax.add_patch(Rectangle((0, 0), L["x_in"], L["y_in"], fc="white", ec="k", lw=0.8))
        ax.set_aspect("equal"); ax.set_xlim(-22, L["x_out"] + 8); ax.set_ylim(-22, L["y_out"] + 6)
        ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)"); ax.set_title(title)

    ax = axes[0]
    att = L["att"]
    ax.add_patch(Rectangle((att["x0"], att["y0"]), p["att_len"], p["att_wid"], fc="#555", ec="k"))
    ax.text((att["x0"] + att["x1"]) / 2, (att["y0"] + att["y1"]) / 2, "AD-USB1AR36G95\nports face -X",
            color="w", ha="center", va="center", fontsize=9)
    for y in att["y_holes"]:
        ax.add_patch(Circle((att["x_hole"], y), 2.5, fc="w", ec="k"))
    ax.add_patch(Rectangle((att["x_sma"] - 3.2, att["y0"] - p["att_sma_len"] - w), 6.4, p["att_sma_len"] + w, fc="gold", ec="k"))
    ax.text(att["x_sma"], -w - p["att_sma_len"] - 3, "RF OUT", ha="center", va="top", fontsize=8)
    ax.add_patch(Rectangle((att["x_sma"] - 3.2, att["y1"]), 6.4, p["att_sma_len"], fc="gold", ec="k"))
    ax.text(att["x_sma"] + 6, att["y1"] + 4, "to bladeRF TX1\n(up through shelf)", fontsize=7, va="center")
    ax.annotate("USB-B", (att["x0"], att["y_usb"]), (att["x0"] - 16, att["y_usb"] + 8), fontsize=8,
                arrowprops=dict(arrowstyle="->"))
    for x0, x1, y0, y1 in (L["shelf_open_usb"], L["shelf_open_sma"]):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="none", ec="r", ls="--"))
    for x, y in L["posts1"]:
        ax.add_patch(Circle((x, y), p["post_d"] / 2, fc="#ccc", ec="k"))

    ax = axes[1]
    pi, sdr, m, sm = L["pi"], L["sdr"], L["pi_map"], L["sdr_map"]
    ax.add_patch(Rectangle((0, 0), p["bay_len"], L["y_in"], fc="#fff3cc", ec="none"))
    ax.text(p["bay_len"] / 2, L["y_in"] - 10, "cable\nbay", ha="center", fontsize=8)
    py0, py1 = L["eth_part_y"]
    ax.add_patch(Rectangle((0, py0), pi["x0"] - 2.5, py1 - py0, fc="k"))
    ax.add_patch(Rectangle((-w, 0), w, py0, fc="w", ec="w"))
    ax.text(-w - 2, py0 / 2, "ETH\ntunnel", ha="right", va="center", fontsize=8)
    ax.add_patch(Rectangle((pi["x0"], pi["y0"]), p["pi_len"], p["pi_wid"], fc="#a8d08d", ec="k"))
    (cx0, cx1), (cy0, cy1) = p["pi_cooler_x"], p["pi_cooler_y"]
    ax.add_patch(Rectangle((m(cx1, 0)[0], m(0, cy0)[1]), cx1 - cx0, cy1 - cy0, fc="#bdbdbd", ec="k", alpha=0.85))
    ax.text(m((cx0 + cx1) / 2, 0)[0], m(0, (cy0 + cy1) / 2)[1], "Pi 5 + cooler", ha="center", va="center", fontsize=9)
    for by in p["pi_usb_y"]:
        ax.add_patch(Rectangle((pi["x0"] - 2.5, m(0, by)[1] - 6.6), 17, 13.2, fc="#333", ec="k"))
    ax.add_patch(Rectangle((pi["x0"] - 2.0, m(0, p["pi_eth_y"])[1] - 8), 21, 16, fc="#333", ec="k"))
    for bx, lab in ((p["pi_usbc_x"], "USB-C"), (p["pi_hdmi_x"][0], "HDMI"), (p["pi_hdmi_x"][1], "HDMI")):
        x = m(bx, 0)[0]
        ax.add_patch(Rectangle((x - 4.5, -1.5), 9, 4, fc="#333", ec="k"))
        ax.text(x, -w - 2, lab, ha="center", va="top", fontsize=8)
    ax.add_patch(Rectangle((sdr["x0"], sdr["y0"]), p["sdr_len"], p["sdr_wid"], fc="#9dc3e6", ec="k"))
    ax.text((sdr["x0"] + sdr["x1"]) / 2, (sdr["y0"] + sdr["y1"]) / 2, "bladeRF 2.0 micro xA9", ha="center",
            va="center", fontsize=10)
    for bx, by in L["sdr_holes"]:
        ax.add_patch(Circle(sm(bx, by), 1.3, fc="w", ec="k"))
    for by, lab in zip(p["sdr_sma_y"], p["sdr_sma_labels"]):
        y = sm(0, by)[1]
        ax.add_patch(Rectangle((sdr["x1"], y - 3.2), p["sdr_sma_len"], 6.4, fc="gold", ec="k"))
        ax.text(sdr["x1"] + 11, y, lab, va="center", fontsize=7)
    uy = sm(0, p["sdr_usbc_y"])[1]
    ax.add_patch(Rectangle((sdr["x0"] - 5, uy - 6.9), 18, 13.8, fc="#2a5caa", ec="k"))
    ax.text(sdr["x0"] - 7, uy, "USB 3.0 B", ha="right", va="center", fontsize=7)
    dy = sm(0, p["sdr_dc_y"])[1]
    ax.add_patch(Rectangle((sdr["x0"] - 3.8, dy - 5.4), 16, 10.8, fc="#222", ec="k"))
    ax.text(sdr["x0"] - 6, dy, "DC", ha="right", va="center", fontsize=7)
    for x0, x1, y0, y1 in (L["shelf_open_usb"], L["shelf_open_sma"]):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="none", ec="r", ls="--"))
    for x, y in L["posts2"]:
        ax.add_patch(Circle((x, y), p["post_d"] / 2, fc="#ccc", ec="k"))

    fig.suptitle(f"Two-storey enclosure, bladeRF variant - outer {L['x_out']:.1f} x {L['y_out']:.1f} x "
                 f"{L['z_top']:.1f} mm (red dashed = cable openings in the shelf)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def build(out_dir: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    L = layout()
    parts = {"lower_box": A.lower_box(L), "tray": A.tray(L), "lid": A.lid(L)}
    for name, part in parts.items():
        assert part.is_watertight, f"{name} is not watertight"
        part.export(os.path.join(out_dir, f"{name}.stl"))
    parts["lid"].copy().rotate(180, axis=(1, 0, 0)).min_to("z", 0).export(
        os.path.join(out_dir, "lid_print_orientation.stl"))

    scene = trimesh.Scene()
    for name, colour in (("lower_box", (195, 195, 200, 255)), ("tray", (215, 215, 220, 255)),
                         ("lid", (235, 235, 235, 255))):
        mesh = parts[name].mesh.copy()
        mesh.visual.face_colors = colour
        scene.add_geometry(mesh, node_name=name)
    for name, ph in placeholders(L).items():
        scene.add_geometry(ph.mesh, node_name=f"device_{name}")
    scene.export(os.path.join(out_dir, "assembly.glb"))
    plan_drawing(L, os.path.join(out_dir, "plan.png"))

    A.export_print_3mf(parts, os.path.join(out_dir, "enclosure_all_parts.3mf"))
    A.export_plate_3mf(parts, os.path.join(out_dir, "enclosure_h2d_plate.3mf"), bed=(350.0, 320.0))

    print(f"outer size: {L['x_out']:.1f} x {L['y_out']:.1f} x {L['z_top']:.1f} mm")
    for name, part in parts.items():
        b = part.bounds
        print(f"{name:10s} {b[1][0]-b[0][0]:6.1f} x {b[1][1]-b[0][1]:6.1f} x {b[1][2]-b[0][2]:5.1f} mm  "
              f"volume {part.volume/1000:6.1f} cm^3  faces {len(part.faces)}")
    return L


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "output_attenuator_bladerf"))
