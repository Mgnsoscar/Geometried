"""
sdr_enclosure.py

Parametric, 3D-printable enclosure for a Raspberry Pi 5 (with the official
Active Cooler) and an Ettus USRP B206mini-i board, built with geomeTried3d.

Run it from the repository root:

    python examples/sdr_enclosure/sdr_enclosure.py [output_dir]

It writes `box.stl`, `lid.stl` (in place) and `lid_print_orientation.stl`
(flipped so it prints flat), an `assembly.glb` preview with coloured
placeholders for the two boards, and a `plan.png` drawing.

Layout (mm; X = length, Y = width, Z = up, origin at the inner floor corner)
-----------------------------------------------------------------------------
Single storey.  The Pi lies along X against the Y = 0 wall so its USB-C power
and both micro-HDMI ports pass through that wall (the PSU plugs straight into
the case).  The B206mini-i lies beside it; its three SMA connectors pass
through the +X end wall.  Both boards' USB edges face an open "cable bay" at
the -X end: the Pi's four USB-A ports and Ethernet are reachable from outside
through the bay, and the SDR's USB-C cable loops round inside the bay into one
of those ports.  A microSD slot in the end wall lets the Pi's card out without
opening the box.

Printed parts: `box` (upright) and `lid` (upside down - its locating lip and
the tabs that close the drop-in connector slots then point up, no supports).

Every dimension lives in PARAMS.
"""

from __future__ import annotations

import math
import os
import sys

import trimesh

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from geomeTried3d import Box, Cylinder, Cone, Structure  # noqa: E402

# --------------------------------------------------------------------------- #
# parameters
# --------------------------------------------------------------------------- #

PARAMS = dict(
    # ---- printing / general -------------------------------------------- #
    wall=3.0,            # side wall thickness
    floor=3.0,
    lid=3.0,
    clear=0.5,           # gap between a board edge and the wall it sits against
    sections=48,         # cylinder facet count

    # ---- Raspberry Pi 5 (raspberry-pi-5-mechanical-drawing.pdf) ----------- #
    pi_len=85.0, pi_wid=56.0, pi_pcb=1.6,
    pi_holes=((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)),  # (x from SD end, y from USB-C edge)
    pi_usbc_x=11.2, pi_hdmi_x=(25.8, 39.2),        # along the USB-C edge, from the SD-card end
    pi_eth_y=10.2, pi_usb_y=(29.1, 47.0),          # along the port edge, from the USB-C edge
    pi_sd_y=(22.0, 35.0),                          # microSD card span along the SD-card edge
    pi_button_y=(12.0, 21.0),                      # power button + UART area on the SD-card edge
    pi_cooler_x=(1.0, 64.5), pi_cooler_y=(6.0, 49.0),   # active cooler footprint on the board
    pi_top_clear=21.0,   # headroom above the PCB (USB-A stacks 15.6, cooler ~14 + fan intake)
    pi_standoff=5.0,
    pi_end_gap=7.5,      # SD-card end to the end wall (card protrudes ~2.5, posts need 7)

    # ---- Ettus B206mini-i (B200mini drawing + B206mini-i spec figure) ------ #
    sdr_len=74.7, sdr_wid=51.0, sdr_pcb=1.6,
    sdr_hole_pitch=(66.0, 43.2), sdr_hole_d=3.175,   # 4 holes, centred on the board
    sdr_sma_y=(11.4, 25.4, 39.4),   # TRX, RX2, REF - from the TRX long edge
    sdr_sma_len=9.6,                # protrusion beyond the PCB edge
    sdr_sma_slot=9.6,               # slot width: lets the SMA hex pass through the wall
    sdr_usbc_y=15.9,                # USB-C on the opposite short edge, from the TRX edge
    sdr_gpio_x=(45.5, 66.0),        # GPIO PicoBlade on the REF-side long edge, from the USB-C edge
    sdr_top=8.7, sdr_under=1.7,
    sdr_standoff=5.0,
    sdr_side_gap=7.0,               # REF-side edge to the +Y wall (room for a corner post)
    gpio_cutout=False,              # optional window in the +Y wall for the GPIO cable

    # ---- layout --------------------------------------------------------- #
    bay_len=45.0,        # open cable bay at the -X end
    board_gap=4.0,       # between the Pi and the SDR, side by side
    bay_top_open=True,   # cut the lid away over the bay (easier to reach the ports)

    # ---- fasteners -------------------------------------------------------- #
    post_d=7.0, post_hole=2.6, post_inset=3.5,     # M3 thread-forming into the corner posts
    screw_d=3.4, csk_d=6.4,                        # M3 countersunk through the lid
    standoff_pi_d=6.5, standoff_pi_hole=2.2,       # M2.5 thread-forming
    standoff_sdr_d=7.0, standoff_sdr_hole=2.6,     # M3 thread-forming

    # ---- openings ---------------------------------------------------------- #
    pi_window_margin=6.0,   # beyond the outer HDMI / USB-C centres
    pi_window_h=10.0,       # open height of the power/HDMI window below the lid tab
    sd_slot_h=3.0,
    vent_hole_d=3.5, vent_pitch=5.5,
    slot_w=2.0, slot_h=12.0, slot_pitch=5.0,
    tab_t=2.5, tab_clear=0.3,
    lip_t=1.5, lip_h=2.0,
)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def P(key):
    return PARAMS[key]


def box(x0, x1, y0, y1, z0, z1) -> Box:
    return Box.from_bounds(((x0, y0, z0), (x1, y1, z1)))


def cyl_z(x, y, z0, z1, d) -> Cylinder:
    return Cylinder(d / 2, z1 - z0, sections=P("sections"), pos=(x, y, (z0 + z1) / 2))


def cyl_x(y, z, x0, x1, d) -> Structure:
    """Cylinder along X (holes through the end walls)."""
    c = Cylinder(d / 2, x1 - x0, sections=P("sections"))
    c.rotate(90, axis=(0, 1, 0))
    return c.translate(((x0 + x1) / 2, y, z))


def u_slot_x(y, z, x0, x1, width, z_top) -> Structure:
    """Drop-in slot through an end wall: round bottom centred on (y, z), open
    up to z_top."""
    return cyl_x(y, z, x0, x1, width).union(
        box(x0, x1, y - width / 2, y + width / 2, z, z_top))


def countersink_down(x, y, z1, d_screw, d_head) -> Structure:
    """90-degree countersink opening upwards, head flush with the face at z1."""
    depth = (d_head - d_screw) / 2
    cone = Cone.frustum(d_screw / 2, d_head / 2, depth, sections=P("sections"))
    return cone.translate((x, y, z1 - depth / 2 + 0.01))


def concat(parts: list) -> Structure:
    """Disjoint cutters as one mesh, no boolean needed."""
    return Structure(trimesh.util.concatenate([p.mesh for p in parts]))


# --------------------------------------------------------------------------- #
# derived layout
# --------------------------------------------------------------------------- #

def layout() -> dict:
    p = PARAMS
    w, f, c = p["wall"], p["floor"], p["clear"]
    L = {}

    # ---- Pi: ports face -X into the bay, USB-C edge on the Y = 0 wall ---- #
    pi_x0 = p["bay_len"] + c
    pi_x1 = pi_x0 + p["pi_len"]
    L["pi"] = dict(x0=pi_x0, x1=pi_x1, y0=c, y1=c + p["pi_wid"],
                   z_pcb0=f + p["pi_standoff"])
    L["pi"]["z_pcb1"] = L["pi"]["z_pcb0"] + p["pi_pcb"]
    L["pi_map"] = lambda bx, by: (pi_x1 - bx, c + by)      # board (x from SD end, y from USB-C edge)

    # ---- SDR: SMAs through the +X wall, TRX edge towards the Pi ---------- #
    L["x_in"] = pi_x1 + p["pi_end_gap"]
    sdr_x1 = L["x_in"] - c
    sdr_x0 = sdr_x1 - p["sdr_len"]
    sdr_y0 = L["pi"]["y1"] + p["board_gap"]
    sdr_y1 = sdr_y0 + p["sdr_wid"]
    L["y_in"] = sdr_y1 + p["sdr_side_gap"]
    L["sdr"] = dict(x0=sdr_x0, x1=sdr_x1, y0=sdr_y0, y1=sdr_y1,
                    z_pcb0=f + p["sdr_standoff"])
    L["sdr"]["z_pcb1"] = L["sdr"]["z_pcb0"] + p["sdr_pcb"]
    L["sdr"]["z_sma"] = L["sdr"]["z_pcb0"] + p["sdr_pcb"] / 2
    L["sdr_map"] = lambda bx, by: (sdr_x0 + bx, sdr_y0 + by)  # board (x from USB-C edge, y from TRX edge)
    px, py = p["sdr_hole_pitch"]
    L["sdr_holes"] = [((p["sdr_len"] - px) / 2 + i * px, (p["sdr_wid"] - py) / 2 + j * py)
                      for i in (0, 1) for j in (0, 1)]

    # ---- heights ---------------------------------------------------------- #
    L["z_floor"] = f
    L["z_rim"] = f + p["pi_standoff"] + p["pi_pcb"] + p["pi_top_clear"]
    L["z_top"] = L["z_rim"] + p["lid"]

    # ---- outer -------------------------------------------------------------- #
    L["x_out"], L["y_out"] = L["x_in"] + 2 * w, L["y_in"] + 2 * w
    i = p["post_inset"]
    L["posts"] = [(i, i), (i, L["y_in"] - i), (L["x_in"] - i, i), (L["x_in"] - i, L["y_in"] - i)]

    xs = [L["pi_map"](bx, 0)[0] for bx in (p["pi_usbc_x"], *p["pi_hdmi_x"])]
    L["pi_window"] = (min(xs) - p["pi_window_margin"], max(xs) + p["pi_window_margin"],
                      L["pi"]["z_pcb1"] - 2.0)
    return L


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #

def main_box(L: dict) -> Structure:
    p = PARAMS
    w, f = p["wall"], p["floor"]
    x_in, y_in, z_rim = L["x_in"], L["y_in"], L["z_rim"]
    pi, sdr = L["pi"], L["sdr"]

    shell = box(-w, x_in + w, -w, y_in + w, 0, z_rim)
    cavity = box(0, x_in, 0, y_in, f, z_rim + 1)
    bay_mouth = box(-w - 1, 0.01, 0, y_in, f, z_rim + 1)       # open -X end
    body = shell.difference(cavity, bay_mouth)

    # lid posts and board standoffs
    adds = [cyl_z(x, y, f - 0.01, z_rim, p["post_d"]) for x, y in L["posts"]]
    adds += [cyl_z(*L["pi_map"](bx, by), f - 0.01, pi["z_pcb0"], p["standoff_pi_d"])
             for bx, by in p["pi_holes"]]
    adds += [cyl_z(*L["sdr_map"](bx, by), f - 0.01, sdr["z_pcb0"], p["standoff_sdr_d"])
             for bx, by in L["sdr_holes"]]
    body = body.union(*adds)

    cutters = []
    for x, y in L["posts"]:
        cutters.append(cyl_z(x, y, z_rim - 12, z_rim + 1, p["post_hole"]))
    for bx, by in p["pi_holes"]:
        cutters.append(cyl_z(*L["pi_map"](bx, by), f + 1, pi["z_pcb0"] + 1, p["standoff_pi_hole"]))
    for bx, by in L["sdr_holes"]:
        cutters.append(cyl_z(*L["sdr_map"](bx, by), f + 1, sdr["z_pcb0"] + 1, p["standoff_sdr_hole"]))

    # SDR SMA drop-in slots through the +X end wall
    for by in p["sdr_sma_y"]:
        _, y = L["sdr_map"](0, by)
        cutters.append(u_slot_x(y, sdr["z_sma"], x_in - 1, x_in + w + 1, p["sdr_sma_slot"], z_rim + 1))

    # Pi power / HDMI window on the Y = 0 wall (drop-in, closed by a lid tab)
    wx0, wx1, wz = L["pi_window"]
    cutters.append(box(wx0, wx1, -w - 1, 1, wz, z_rim + 1))

    # microSD slot in the +X end wall, just under the Pi PCB
    y0, y1 = (L["pi_map"](0, y)[1] for y in p["pi_sd_y"])
    cutters.append(box(x_in - 1, x_in + w + 1, y0, y1, pi["z_pcb0"] - p["sd_slot_h"], pi["z_pcb0"] + 0.4))

    # poke-slot for the Pi's power button (on the SD-card edge, just above the PCB)
    b0, b1 = (L["pi_map"](0, y)[1] for y in p["pi_button_y"])
    cutters.append(box(x_in - 1, x_in + w + 1, b0, b1, pi["z_pcb1"] + 0.3, pi["z_pcb1"] + 3.5))

    # optional GPIO window in the +Y wall for the SDR's PicoBlade cable
    if p["gpio_cutout"]:
        gx0, gx1 = (L["sdr_map"](bx, 0)[0] for bx in p["sdr_gpio_x"])
        cutters.append(box(gx0 - 2, gx1 + 2, y_in - 1, y_in + w + 1, sdr["z_pcb1"] - 1, sdr["z_pcb1"] + 12))

    # exhaust slots: Y = 0 wall either side of the window, +X end wall beside the
    # Pi, +Y wall along the SDR
    z0 = pi["z_pcb1"] + 1.0
    sw, sp = p["slot_w"], p["slot_pitch"]

    def slot_run(xa, xb, make):
        x = xa
        while x + sw <= xb:
            cutters.append(make(x))
            x += sp

    slot_run(pi["x0"] + 4, wx0 - 3, lambda x: box(x, x + sw, -w - 1, 1, z0, z0 + p["slot_h"]))
    slot_run(wx1 + 3, pi["x1"] - 2, lambda x: box(x, x + sw, -w - 1, 1, z0, z0 + p["slot_h"]))
    slot_run(sdr["x0"] + 6, sdr["x1"] - 6, lambda x: box(x, x + sw, y_in - 1, y_in + w + 1, z0, z0 + p["slot_h"]))
    y = pi["y0"] + 3
    while y + sw <= pi["y1"] - 3:
        if not (y0 - 3 <= y <= y1 + 1):      # keep clear of the SD slot
            cutters.append(box(x_in - 1, x_in + w + 1, y, y + sw, z0, z0 + p["slot_h"]))
        y += sp

    return body.difference(*cutters)


def lid(L: dict) -> Structure:
    """Modelled in place. Printed upside down: lip and tabs point up."""
    p = PARAMS
    w, c = p["wall"], p["tab_clear"]
    x_in, y_in = L["x_in"], L["y_in"]
    z0, z1 = L["z_rim"], L["z_top"]
    lip_t, lip_h = p["lip_t"], p["lip_h"]
    sdr = L["sdr"]

    plate = box(-w, x_in + w, -w, y_in + w, z0, z1)

    # locating lip inside the three walled sides
    lip = box(c, x_in - c, c, y_in - c, z0 - lip_h, z0 + 0.01).difference(
        box(-1, x_in - c - lip_t, c + lip_t, y_in - c - lip_t, z0 - lip_h - 1, z0 + 1))
    body = plate.union(lip)

    # tabs closing the SMA slots and the Pi window
    tab_w = p["sdr_sma_slot"] - 2 * c
    z_tab = sdr["z_sma"] + p["sdr_sma_slot"] / 2 + 1.5
    tabs = [box(x_in + c, x_in + c + p["tab_t"], y - tab_w / 2, y + tab_w / 2, z_tab, z0 + 0.01)
            for y in (L["sdr_map"](0, by)[1] for by in p["sdr_sma_y"])]
    wx0, wx1, wz = L["pi_window"]
    tabs.append(box(wx0 + c, wx1 - c, -w + c, -w + c + p["tab_t"], wz + p["pi_window_h"], z0 + 0.01))
    body = body.union(*tabs)

    cutters = []
    for x, y in L["posts"]:
        cutters.append(cyl_z(x, y, z0 - lip_h - 1, z1 + 1, p["screw_d"]))
        cutters.append(countersink_down(x, y, z1, p["screw_d"], p["csk_d"]))
        cutters.append(cyl_z(x, y, z0 - lip_h - 1, z0 + 0.5, p["post_d"] + 1.0))   # lip clears the posts

    if p["bay_top_open"]:
        cutters.append(box(p["post_inset"] + p["post_d"] / 2 + 1.5, p["bay_len"] - 1.0,
                           p["post_inset"] + p["post_d"] / 2 + 1.5, y_in - p["post_inset"] - p["post_d"] / 2 - 1.5,
                           z0 - lip_h - 1, z1 + 1))

    # fan intake grille over the active cooler
    (cx0, cx1), (cy0, cy1) = p["pi_cooler_x"], p["pi_cooler_y"]
    xa, ya = L["pi_map"](cx1, cy0)
    xb, yb = L["pi_map"](cx0, cy1)
    pitch, d = p["vent_pitch"], p["vent_hole_d"]
    row = pitch * math.sqrt(3) / 2
    holes = []
    j = 0
    while ya + d / 2 + j * row + d / 2 <= yb:
        y = ya + d / 2 + j * row
        x = xa + d / 2 + (pitch / 2 if j % 2 else 0.0)
        while x + d / 2 <= xb:
            holes.append(cyl_z(x, y, z0 - 1, z1 + 1, d))
            x += pitch
        j += 1
    cutters.append(concat(holes))

    return body.difference(*cutters)


# --------------------------------------------------------------------------- #
# device placeholders for the preview
# --------------------------------------------------------------------------- #

def placeholders(L: dict) -> dict:
    p = PARAMS
    pi, sdr = L["pi"], L["sdr"]
    m = L["pi_map"]

    pcb = box(pi["x0"], pi["x1"], pi["y0"], pi["y1"], pi["z_pcb0"], pi["z_pcb1"])
    (cx0, cx1), (cy0, cy1) = p["pi_cooler_x"], p["pi_cooler_y"]
    cooler = box(m(cx1, 0)[0], m(cx0, 0)[0], m(0, cy0)[1], m(0, cy1)[1], pi["z_pcb1"], pi["z_pcb1"] + 14.0)
    usb = [box(pi["x0"] - 2.5, pi["x0"] + 17.5, m(0, y)[1] - 6.6, m(0, y)[1] + 6.6, pi["z_pcb1"], pi["z_pcb1"] + 15.6)
           for y in p["pi_usb_y"]]
    ey = m(0, p["pi_eth_y"])[1]
    eth = box(pi["x0"] - 2.0, pi["x0"] + 21.0, ey - 8, ey + 8, pi["z_pcb1"], pi["z_pcb1"] + 13.6)
    pi_ph = pcb.union(cooler, *usb, eth).set_color(70, 150, 70)

    spcb = box(sdr["x0"], sdr["x1"], sdr["y0"], sdr["y1"], sdr["z_pcb0"], sdr["z_pcb1"])
    smas = [cyl_x(L["sdr_map"](0, by)[1], sdr["z_sma"], sdr["x1"] - 2, sdr["x1"] + p["sdr_sma_len"], 6.35)
            for by in p["sdr_sma_y"]]
    uy = L["sdr_map"](0, p["sdr_usbc_y"])[1]
    usbc_plug = box(sdr["x0"] - 24, sdr["x0"], uy - 5, uy + 5, sdr["z_pcb1"], sdr["z_pcb1"] + 6.5)
    sdr_ph = spcb.union(*smas, usbc_plug).set_color(50, 100, 190)
    return {"pi": pi_ph, "sdr": sdr_ph}


# --------------------------------------------------------------------------- #
# plan drawing
# --------------------------------------------------------------------------- #

def plan_drawing(L: dict, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    p = PARAMS
    w = p["wall"]
    pi, sdr, m, sm = L["pi"], L["sdr"], L["pi_map"], L["sdr_map"]

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.add_patch(Rectangle((-w, -w), L["x_out"], L["y_out"], fc="#e6e6e6", ec="k", lw=1.5))
    ax.add_patch(Rectangle((0, 0), L["x_in"], L["y_in"], fc="white", ec="k", lw=0.8))
    ax.add_patch(Rectangle((-w - 0.6, 0), w + 0.6, L["y_in"], fc="white", ec="white"))   # open end
    ax.add_patch(Rectangle((0, 0), p["bay_len"], L["y_in"], fc="#fff3cc", ec="none"))
    ax.text(p["bay_len"] / 2, L["y_in"] - 12, "open cable bay", ha="center", fontsize=9)

    # Pi
    ax.add_patch(Rectangle((pi["x0"], pi["y0"]), p["pi_len"], p["pi_wid"], fc="#a8d08d", ec="k"))
    (cx0, cx1), (cy0, cy1) = p["pi_cooler_x"], p["pi_cooler_y"]
    ax.add_patch(Rectangle((m(cx1, 0)[0], m(0, cy0)[1]), cx1 - cx0, cy1 - cy0, fc="#bdbdbd", ec="k", alpha=0.85))
    ax.text(m((cx0 + cx1) / 2, 0)[0], m(0, (cy0 + cy1) / 2)[1], "Pi 5 + active cooler\n(intake grille in lid)",
            ha="center", va="center", fontsize=9)
    for bx, by in p["pi_holes"]:
        ax.add_patch(Circle(m(bx, by), 1.35, fc="w", ec="k"))
    for by in p["pi_usb_y"]:
        ax.add_patch(Rectangle((pi["x0"] - 2.5, m(0, by)[1] - 6.6), 17, 13.2, fc="#333", ec="k"))
    ax.add_patch(Rectangle((pi["x0"] - 2.0, m(0, p["pi_eth_y"])[1] - 8), 21, 16, fc="#333", ec="k"))
    ax.text(pi["x0"] - 4, pi["y0"] + 3, "USB x4 + Ethernet face the bay", ha="right", va="bottom",
            fontsize=8, rotation=90)
    for bx, lab in ((p["pi_usbc_x"], "USB-C\npower"), (p["pi_hdmi_x"][0], "HDMI"), (p["pi_hdmi_x"][1], "HDMI")):
        x = m(bx, 0)[0]
        ax.add_patch(Rectangle((x - 4.5, -1.5), 9, 4, fc="#333", ec="k"))
        ax.text(x, -w - 2, lab, ha="center", va="top", fontsize=8)
    y0, y1 = (m(0, y)[1] for y in p["pi_sd_y"])
    ax.add_patch(Rectangle((L["x_in"], y0), w, y1 - y0, fc="w", ec="k"))
    ax.text(L["x_in"] + w + 1, (y0 + y1) / 2, "microSD slot", va="center", fontsize=8)

    # SDR
    ax.add_patch(Rectangle((sdr["x0"], sdr["y0"]), p["sdr_len"], p["sdr_wid"], fc="#9dc3e6", ec="k"))
    ax.text((sdr["x0"] + sdr["x1"]) / 2, (sdr["y0"] + sdr["y1"]) / 2, "B206mini-i", ha="center", va="center",
            fontsize=10)
    for bx, by in L["sdr_holes"]:
        ax.add_patch(Circle(sm(bx, by), 1.6, fc="w", ec="k"))
    for by, lab in zip(p["sdr_sma_y"], ("TRX", "RX2", "REF")):
        y = sm(0, by)[1]
        ax.add_patch(Rectangle((sdr["x1"], y - 3.2), p["sdr_sma_len"], 6.4, fc="gold", ec="k"))
        ax.text(sdr["x1"] + w + 8, y, lab, va="center", fontsize=8)
    uy = sm(0, p["sdr_usbc_y"])[1]
    ax.add_patch(Rectangle((sdr["x0"] - 24, uy - 5), 24, 10, fc="#333", ec="k", alpha=0.5))
    ax.text(sdr["x0"] - 12, uy + 8, "USB-C plug\n(cable loops into the bay)", ha="center", fontsize=7)
    gx0, gx1 = (sm(bx, 0)[0] for bx in p["sdr_gpio_x"])
    ax.add_patch(Rectangle((gx0, sdr["y1"] - 3), gx1 - gx0, 3, fc="#555", ec="k"))
    ax.text((gx0 + gx1) / 2, sdr["y1"] + 1, "GPIO", ha="center", va="bottom", fontsize=7)

    for x, y in L["posts"]:
        ax.add_patch(Circle((x, y), p["post_d"] / 2, fc="#ccc", ec="k"))

    ax.set_aspect("equal")
    ax.set_xlim(-18, L["x_out"] + 22)
    ax.set_ylim(-22, L["y_out"] + 6)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f"Plan view - outer {L['x_out']:.1f} x {L['y_out']:.1f} x {L['z_top']:.1f} mm, "
                 f"bay open at X = 0 (lid shown removed)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def build(out_dir: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    L = layout()

    parts = {"box": main_box(L), "lid": lid(L)}
    for name, part in parts.items():
        assert part.is_watertight, f"{name} is not watertight"
        part.export(os.path.join(out_dir, f"{name}.stl"))

    lid_print = parts["lid"].copy().rotate(180, axis=(1, 0, 0)).min_to("z", 0)
    lid_print.export(os.path.join(out_dir, "lid_print_orientation.stl"))

    scene = trimesh.Scene()
    for name, colour in (("box", (205, 205, 205, 255)), ("lid", (235, 235, 235, 110))):
        mesh = parts[name].mesh.copy()
        mesh.visual.face_colors = colour
        scene.add_geometry(mesh, node_name=name)
    for name, ph in placeholders(L).items():
        scene.add_geometry(ph.mesh, node_name=f"device_{name}")
    scene.export(os.path.join(out_dir, "assembly.glb"))

    plan_drawing(L, os.path.join(out_dir, "plan.png"))

    print(f"outer size: {L['x_out']:.1f} x {L['y_out']:.1f} x {L['z_top']:.1f} mm")
    for name, part in parts.items():
        b = part.bounds
        print(f"{name:4s} {b[1][0]-b[0][0]:6.1f} x {b[1][1]-b[0][1]:6.1f} x {b[1][2]-b[0][2]:5.1f} mm  "
              f"volume {part.volume/1000:6.1f} cm^3  faces {len(part.faces)}")
    return L


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "output"))
