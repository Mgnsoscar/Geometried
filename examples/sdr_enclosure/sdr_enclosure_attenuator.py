"""
sdr_enclosure_attenuator.py

Two-storey, 3D-printable enclosure for a Raspberry Pi 5 (Active Cooler), an
Ettus USRP B206mini-i board and an Adaura AD-USB1AR36G95 attenuator, built
with geomeTried3d. Companion to sdr_enclosure.py (which has no attenuator);
the board dimensions and the geometry helpers are imported from there.

    python examples/sdr_enclosure/sdr_enclosure_attenuator.py [output_dir]

RF path: SDR TRX -> (internal SMA jumper) -> attenuator -> output SMA on the
front wall. Only the attenuator output, the Pi's power/HDMI and the Pi's
Ethernet port are reachable from outside. Everything else is internal.

Layout (mm; X = length, Y = width, Z = up, origin at the lower inner corner)
-----------------------------------------------------------------------------
Lower box  - attenuator lying along X against the front wall. Its port face
             looks at the -X end wall across a short channel for a right-angle
             USB-B plug. Output SMA through the front wall; the other SMA
             points +Y inside, where a right-angle plug turns up through an
             opening in the shelf.
Tray       - Pi along X against the front wall (USB-C + HDMI through it),
             SDR beside it. Both boards' USB edges face an enclosed cable bay
             at the -X end. The SDR's TRX SMA points +X, directly above the
             attenuator's internal SMA, so the jumper is a short vertical loop
             through the shelf. A partition in the bay's front corner makes a
             tunnel from the Pi's Ethernet jack out through the end wall.
Lid        - closes the tray; grille over the cooler; tabs close the drop-in
             window for the Pi's connectors.

Printed parts: lower_box, tray (both upright), lid (upside down).
"""

from __future__ import annotations

import os
import sys

import trimesh

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from geomeTried3d import Structure  # noqa: E402
import sdr_enclosure as base  # noqa: E402
from sdr_enclosure import (  # noqa: E402
    box, cyl_z, cyl_x, cyl_y, countersink_down, countersink_up, concat, vent_grid,
    rrect, filleted_slab,
)

P0 = base.PARAMS

PARAMS = dict(
    P0,   # board dimensions, wall/floor/lid thicknesses, fasteners, vents ...

    # ---- Adaura AD-USB1AR36G95 (datasheet drawing) ----------------------- #
    att_len=110.0, att_wid=89.3, att_hgt=25.7,
    att_sma_from_port_face=90.7,    # along the length
    att_sma_z=9.0,                  # SMA axis above the bottom face
    att_sma_len=10.35,              # protrusion beyond the body
    att_hole_from_port_face=57.1, att_hole_pitch=69.3,   # 2x Ø5 through holes
    att_usb_from_left=34.1,         # USB-B centre, seen facing the port face
    att_sma_hole=7.2,               # round hole for the output SMA barrel
    att_screw_d=4.4, att_csk_d=8.8, # M4 countersunk from below
    att_head_clear=1.3,             # over the attenuator, under the shelf

    # ---- layout ------------------------------------------------------------ #
    plugs="straight",       # "straight" or "right_angle": sets the three run-out spaces below
    board_gap=4.0,          # Pi to SDR
    sdr_side_gap=0.5,       # SDR to the back wall (minimum)
    pi_end_gap=7.5,         # Pi SD-card end to the +X corner posts
    shelf=3.0,              # tray floor thickness

    # ---- styling ------------------------------------------------------------- #
    corner_radius=8.0,      # outer vertical corners (inner cavity corners are corner_radius - wall)
    edge_fillet=2.0,        # lid top edge and lower-box bottom edge
    seam_chamfer=0.6,       # small chamfer where the three parts meet (shadow lines)

    # ---- Ethernet tunnel ---------------------------------------------------- #
    eth_tunnel_w=20.0,      # inner width, measured from the front wall
    eth_partition=1.6,
    eth_open_z=(-1.5, 18.0),  # opening in the end wall, relative to the Pi PCB top (jack is 13.6 tall)
)

# Run-out space in front of each connector: plug body plus the cable's bend.
PLUG_PRESETS = {
    "straight": dict(
        usb_b_channel=45.0,   # lower floor, -X end: 32 mm USB-B plug + bend up to the shelf opening
        bay_len=40.0,         # tray, -X end: USB-A and USB-C plugs + bend along the bay
        sma_strip=32.0,       # beyond the attenuator's internal SMA tip: 20 mm plug + bend up
        sma_channel=30.0,     # beyond the SDR's TRX tip on the tray: plug + bend down
    ),
    "right_angle": dict(usb_b_channel=16.5, bay_len=28.0, sma_strip=9.0, sma_channel=9.0),
}
PARAMS.update(PLUG_PRESETS[PARAMS["plugs"]])


# --------------------------------------------------------------------------- #
# layout
# --------------------------------------------------------------------------- #

def layout() -> dict:
    p = PARAMS
    w, f, c = p["wall"], p["floor"], p["clear"]
    L = {}

    # ---- storeys ----------------------------------------------------------- #
    L["z_floor1"] = f
    L["z_rim1"] = f + p["att_hgt"] + p["att_head_clear"]
    L["z_floor2"] = L["z_rim1"] + p["shelf"]
    L["z_rim2"] = L["z_floor2"] + p["pi_standoff"] + p["pi_pcb"] + p["pi_top_clear"]
    L["z_top"] = L["z_rim2"] + p["lid"]

    # ---- lower floor: attenuator ------------------------------------------- #
    att_x0 = p["usb_b_channel"]
    att_x1 = att_x0 + p["att_len"]
    L["x_in"] = att_x1 + c + p["post_d"]            # room for the +X corner posts
    L["att"] = dict(x0=att_x0, x1=att_x1, y0=c, y1=c + p["att_wid"],
                    z0=f, z1=f + p["att_hgt"], z_sma=f + p["att_sma_z"],
                    x_sma=att_x0 + p["att_sma_from_port_face"],
                    x_hole=att_x0 + p["att_hole_from_port_face"])
    mid = c + p["att_wid"] / 2
    L["att"]["y_holes"] = (mid - p["att_hole_pitch"] / 2, mid + p["att_hole_pitch"] / 2)
    # port face looks -X; seen from the channel, "left" is +Y
    L["att"]["y_usb"] = L["att"]["y1"] - p["att_usb_from_left"]
    L["att"]["y_sma_in_tip"] = L["att"]["y1"] + p["att_sma_len"]

    # ---- tray: Pi (front) and SDR (back), USB edges facing the bay --------- #
    pi_x0 = p["bay_len"] + c
    pi_x1 = pi_x0 + p["pi_len"]
    L["pi"] = dict(x0=pi_x0, x1=pi_x1, y0=c, y1=c + p["pi_wid"],
                   z_pcb0=L["z_floor2"] + p["pi_standoff"])
    L["pi"]["z_pcb1"] = L["pi"]["z_pcb0"] + p["pi_pcb"]
    L["pi_map"] = lambda bx, by: (pi_x1 - bx, c + by)

    sdr_x0 = pi_x0
    sdr_x1 = sdr_x0 + p["sdr_len"]
    sdr_y0 = L["pi"]["y1"] + p["board_gap"]
    sdr_y1 = sdr_y0 + p["sdr_wid"]
    L["y_in"] = max(sdr_y1 + p["sdr_side_gap"], L["att"]["y_sma_in_tip"] + p["sma_strip"])
    L["sdr"] = dict(x0=sdr_x0, x1=sdr_x1, y0=sdr_y0, y1=sdr_y1,
                    z_pcb0=L["z_floor2"] + p["sdr_standoff"])
    L["sdr"]["z_pcb1"] = L["sdr"]["z_pcb0"] + p["sdr_pcb"]
    L["sdr"]["z_sma"] = L["sdr"]["z_pcb0"] + p["sdr_pcb"] / 2
    # TRX edge towards +Y so the TRX SMA sits over the attenuator's internal SMA
    L["sdr_map"] = lambda bx, by: (sdr_x0 + bx, sdr_y1 - by)
    px, py = p["sdr_hole_pitch"]
    L["sdr_holes"] = [((p["sdr_len"] - px) / 2 + i * px, (p["sdr_wid"] - py) / 2 + j * py)
                      for i in (0, 1) for j in (0, 1)]
    L["sdr"]["x_sma_tip"] = sdr_x1 + p["sdr_sma_len"]
    L["sdr"]["y_trx"] = L["sdr_map"](0, p["sdr_sma_y"][0])[1]

    assert pi_x1 + p["pi_end_gap"] <= L["x_in"], "Pi does not fit in front of the +X posts"
    L["x_in"] = max(L["x_in"], L["sdr"]["x_sma_tip"] + p["sma_channel"] + p["post_inset"])

    # ---- outer, posts, openings -------------------------------------------- #
    L["x_out"], L["y_out"] = L["x_in"] + 2 * w, L["y_in"] + 2 * w
    i = p["post_inset"]
    L["posts1"] = [(i, i), (i, L["y_in"] - i), (L["x_in"] - i, i), (L["x_in"] - i, L["y_in"] - i)]
    # the tray's front-left post would sit in the Ethernet tunnel; move it behind the partition
    L["eth_part_y"] = (p["eth_tunnel_w"], p["eth_tunnel_w"] + p["eth_partition"])
    L["posts2"] = [(i, L["eth_part_y"][1] + i + 0.4), (i, L["y_in"] - i),
                   (L["x_in"] - i, i), (L["x_in"] - i, L["y_in"] - i)]

    xs = [L["pi_map"](bx, 0)[0] for bx in (p["pi_usbc_x"], *p["pi_hdmi_x"])]
    L["pi_window"] = (min(xs) - p["pi_window_margin"], max(xs) + p["pi_window_margin"],
                      L["pi"]["z_pcb1"] - 2.0)

    # shelf openings: USB-B cable up from the -X channel; SMA jumper over the attenuator's SMA
    L["shelf_open_usb"] = (2.0, p["usb_b_channel"] - 1.5, L["posts2"][0][1] + i + 1.0, L["att"]["y1"])
    L["shelf_open_sma"] = (min(L["att"]["x_sma"] - 4.0, L["sdr"]["x_sma_tip"] + 2.0),
                           L["x_in"] - i - p["post_d"] / 2 - 1.5,
                           L["att"]["y1"] - 2.0, L["y_in"] - i - p["post_d"] / 2 - 1.5)
    return L


# --------------------------------------------------------------------------- #
# parts
# --------------------------------------------------------------------------- #

def lower_box(L: dict) -> Structure:
    p = PARAMS
    w, f = p["wall"], p["floor"]
    x_in, y_in, z_rim = L["x_in"], L["y_in"], L["z_rim1"]
    att = L["att"]

    R, Ri = p["corner_radius"], p["corner_radius"] - w
    shell = filleted_slab(-w, x_in + w, -w, y_in + w, 0, z_rim, R,
                          fillet_bottom=p["edge_fillet"], chamfer_top=p["seam_chamfer"])
    body = shell.difference(rrect(0, x_in, 0, y_in, f, z_rim + 1, Ri))
    body = body.union(*[cyl_z(x, y, f - 0.01, z_rim, p["post_d"]) for x, y in L["posts1"]])

    cutters = [cyl_z(x, y, z_rim - 12, z_rim + 1, p["post_hole"]) for x, y in L["posts1"]]
    # attenuator output SMA through the front wall (slide the attenuator -Y to engage it)
    cutters.append(cyl_y(att["x_sma"], att["z_sma"], -w - 1, 1, p["att_sma_hole"]))
    # M4 countersunk from below through the attenuator's two Ø5 holes
    for y in att["y_holes"]:
        cutters.append(cyl_z(att["x_hole"], y, -1, f + 1, p["att_screw_d"]))
        cutters.append(countersink_up(att["x_hole"], y, 0, p["att_screw_d"], p["att_csk_d"]))
    # a few exhaust slots in the back wall
    z0 = f + 6
    x = att["x0"] + 10
    while x + p["slot_w"] <= att["x1"] - 10:
        cutters.append(box(x, x + p["slot_w"], y_in - 1, y_in + w + 1, z0, z0 + p["slot_h"]))
        x += p["slot_pitch"] * 2
    return body.difference(*cutters)


def tray(L: dict) -> Structure:
    p = PARAMS
    w, c = p["wall"], p["clear"]
    x_in, y_in = L["x_in"], L["y_in"]
    z0, z_fl, z_rim = L["z_rim1"], L["z_floor2"], L["z_rim2"]
    pi, sdr = L["pi"], L["sdr"]

    R, Ri = p["corner_radius"], p["corner_radius"] - w
    shell = filleted_slab(-w, x_in + w, -w, y_in + w, z0, z_rim, R,
                          chamfer_top=p["seam_chamfer"], chamfer_bottom=p["seam_chamfer"])
    body = shell.difference(rrect(0, x_in, 0, y_in, z_fl, z_rim + 1, Ri))

    adds = [cyl_z(x, y, z0, z_rim, p["post_d"]) for x, y in L["posts2"]]
    adds += [cyl_z(*L["pi_map"](bx, by), z_fl - 0.01, pi["z_pcb0"], p["standoff_pi_d"]) for bx, by in p["pi_holes"]]
    adds += [cyl_z(*L["sdr_map"](bx, by), z_fl - 0.01, sdr["z_pcb0"], p["standoff_sdr_d"]) for bx, by in L["sdr_holes"]]
    # Ethernet tunnel partition, from the end wall to just short of the Pi's connectors
    py0, py1 = L["eth_part_y"]
    adds.append(box(-0.01, pi["x0"] - 2.5, py0, py1, z_fl - 0.01, z_rim))
    body = body.union(*adds)

    cutters = [cyl_z(x, y, z_rim - 12, z_rim + 1, p["post_hole"]) for x, y in L["posts2"]]
    cutters += [cyl_z(*L["pi_map"](bx, by), z_fl + 1, pi["z_pcb0"] + 1, p["standoff_pi_hole"]) for bx, by in p["pi_holes"]]
    cutters += [cyl_z(*L["sdr_map"](bx, by), z_fl + 1, sdr["z_pcb0"] + 1, p["standoff_sdr_hole"]) for bx, by in L["sdr_holes"]]
    # screws down into the lower box's posts
    for x, y in L["posts1"]:
        cutters.append(cyl_z(x, y, z0 - 1, z_fl + 1, p["screw_d"]))
        cutters.append(countersink_down(x, y, z_fl, p["screw_d"], p["csk_d"]))
    # shelf openings for the cables
    for x0, x1, y0, y1 in (L["shelf_open_usb"], L["shelf_open_sma"]):
        cutters.append(box(x0, x1, y0, y1, z0 - 1, z_fl + 1))
    # Ethernet opening in the -X end wall, cut through the rounded corner too
    ez0, ez1 = (pi["z_pcb1"] + dz for dz in p["eth_open_z"])
    cutters.append(box(-w - 1, Ri + 1, 0, py0, ez0, ez1))
    # Pi power / HDMI window on the front wall
    wx0, wx1, wz = L["pi_window"]
    cutters.append(box(wx0, wx1, -w - 1, 1, wz, z_rim + 1))
    # exhaust slots: front wall beside the window, +X end wall by the Pi, back wall along the SDR
    zs0 = pi["z_pcb1"] + 1.0
    sw, sp = p["slot_w"], p["slot_pitch"]
    x = pi["x0"] + 4
    while x + sw <= wx0 - 3:
        cutters.append(box(x, x + sw, -w - 1, 1, zs0, zs0 + p["slot_h"]))
        x += sp
    x = sdr["x0"] + 6
    while x + sw <= sdr["x1"] - 6:
        cutters.append(box(x, x + sw, y_in - 1, y_in + w + 1, zs0, zs0 + p["slot_h"]))
        x += sp
    y = max(pi["y0"] + 3, Ri + 1)          # keep the end-wall slots out of the rounded corner
    while y + sw <= pi["y1"] - 3:
        cutters.append(box(x_in - 1, x_in + w + 1, y, y + sw, zs0, zs0 + p["slot_h"]))
        y += sp
    return body.difference(*cutters)


def lid(L: dict) -> Structure:
    """Modelled in place; print upside down."""
    p = PARAMS
    w, c = p["wall"], p["tab_clear"]
    x_in, y_in = L["x_in"], L["y_in"]
    z0, z1 = L["z_rim2"], L["z_top"]
    lip_t, lip_h = p["lip_t"], p["lip_h"]

    R, Ri = p["corner_radius"], p["corner_radius"] - w
    plate = filleted_slab(-w, x_in + w, -w, y_in + w, z0, z1, R,
                          fillet_top=p["edge_fillet"], chamfer_bottom=p["seam_chamfer"])
    lip = rrect(c, x_in - c, c, y_in - c, z0 - lip_h, z0 + 0.01, Ri - c).difference(
        rrect(c + lip_t, x_in - c - lip_t, c + lip_t, y_in - c - lip_t, z0 - lip_h - 1, z0 + 1, Ri - c - lip_t))
    body = plate.union(lip)
    wx0, wx1, wz = L["pi_window"]
    body = body.union(box(wx0 + c, wx1 - c, -w + c, -w + c + p["tab_t"], wz + p["pi_window_h"], z0 + 0.01))

    cutters = []
    for x, y in L["posts2"]:
        cutters.append(cyl_z(x, y, z0 - lip_h - 1, z1 + 1, p["screw_d"]))
        cutters.append(countersink_down(x, y, z1, p["screw_d"], p["csk_d"]))
        cutters.append(cyl_z(x, y, z0 - lip_h - 1, z0 + 0.5, p["post_d"] + 1.0))
    # the lip must clear the Ethernet partition
    py0, py1 = L["eth_part_y"]
    cutters.append(box(-1, L["pi"]["x0"], py0 - c, py1 + c, z0 - lip_h - 1, z0 + 0.5))
    cutters.append(concat(vent_grid(L, z0 - 1, z1 + 1)))
    return body.difference(*cutters)


# --------------------------------------------------------------------------- #
# preview placeholders
# --------------------------------------------------------------------------- #

def placeholders(L: dict) -> dict:
    p = PARAMS
    out = base.placeholders_boards(L, PARAMS)
    att = L["att"]
    body = box(att["x0"], att["x1"], att["y0"], att["y1"], att["z0"], att["z1"])
    sma_out = cyl_y(att["x_sma"], att["z_sma"], att["y0"] - p["att_sma_len"], att["y0"] + 1, 6.35)
    sma_in = cyl_y(att["x_sma"], att["z_sma"], att["y1"] - 1, att["y_sma_in_tip"], 6.35)
    out["att"] = body.union(sma_out, sma_in).set_color(60, 60, 65)

    if p["plugs"] == "straight":
        # straight plug bodies, to prove the run-out spaces: USB-B on the
        # attenuator, SMA on both ends of the jumper, USB-A x2 on the Pi
        pi, sdr = L["pi"], L["sdr"]
        usb_b = box(att["x0"] - 32, att["x0"], att["y_usb"] - 6, att["y_usb"] + 6,
                    att["z0"] + 9, att["z0"] + 20)
        sma_a = cyl_y(att["x_sma"], att["z_sma"], att["y_sma_in_tip"] - 4, att["y_sma_in_tip"] + 18, 8.0)
        sma_s = cyl_x(sdr["y_trx"], sdr["z_sma"], sdr["x_sma_tip"] - 4, sdr["x_sma_tip"] + 18, 8.0)
        usb_a = [box(pi["x0"] - 2.5 - 20, pi["x0"] - 2.5, L["pi_map"](0, y)[1] - 7.5, L["pi_map"](0, y)[1] + 7.5,
                     pi["z_pcb1"] + 1, pi["z_pcb1"] + 7) for y in p["pi_usb_y"]]
        out["plugs"] = usb_b.union(sma_a, sma_s, *usb_a).set_color(230, 140, 40)
    return out


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
    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5))
    for ax, title in zip(axes, ("Lower box (attenuator)", "Tray (Pi 5 + B206mini-i)")):
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
    ax.text(att["x_sma"] + 6, att["y1"] + 4, "to SDR TRX\n(up through shelf)", fontsize=7, va="center")
    ax.annotate("USB-B", (att["x0"], att["y_usb"]), (att["x0"] - 16, att["y_usb"] + 8), fontsize=8,
                arrowprops=dict(arrowstyle="->"))
    for x0, x1, y0, y1 in (L["shelf_open_usb"], L["shelf_open_sma"]):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="none", ec="r", ls="--"))
    ax.text(L["shelf_open_usb"][0] + 1, L["shelf_open_usb"][3] + 1, "shelf\nopenings", color="r", fontsize=7)
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
    ax.text((sdr["x0"] + sdr["x1"]) / 2, (sdr["y0"] + sdr["y1"]) / 2, "B206mini-i", ha="center", va="center", fontsize=10)
    for by, lab in zip(p["sdr_sma_y"], ("TRX", "RX2", "REF")):
        y = sm(0, by)[1]
        ax.add_patch(Rectangle((sdr["x1"], y - 3.2), p["sdr_sma_len"], 6.4, fc="gold", ec="k"))
        ax.text(sdr["x1"] + 11, y, lab, va="center", fontsize=7)
    uy = sm(0, p["sdr_usbc_y"])[1]
    ax.add_patch(Rectangle((sdr["x0"] - 24, uy - 5), 24, 10, fc="#333", ec="k", alpha=0.5))
    for x0, x1, y0, y1 in (L["shelf_open_usb"], L["shelf_open_sma"]):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="none", ec="r", ls="--"))
    for x, y in L["posts2"]:
        ax.add_patch(Circle((x, y), p["post_d"] / 2, fc="#ccc", ec="k"))

    fig.suptitle(f"Two-storey enclosure - outer {L['x_out']:.1f} x {L['y_out']:.1f} x {L['z_top']:.1f} mm "
                 "(red dashed = cable openings in the shelf)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def build(out_dir: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    L = layout()
    parts = {"lower_box": lower_box(L), "tray": tray(L), "lid": lid(L)}
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
    export_print_3mf(parts, os.path.join(out_dir, "enclosure_all_parts.3mf"))
    export_plate_3mf(parts, os.path.join(out_dir, "enclosure_h2d_plate.3mf"), bed=(350.0, 320.0))

    print(f"outer size: {L['x_out']:.1f} x {L['y_out']:.1f} x {L['z_top']:.1f} mm")
    for name, part in parts.items():
        b = part.bounds
        print(f"{name:10s} {b[1][0]-b[0][0]:6.1f} x {b[1][1]-b[0][1]:6.1f} x {b[1][2]-b[0][2]:5.1f} mm  "
              f"volume {part.volume/1000:6.1f} cm^3  faces {len(part.faces)}")
    return L


def export_print_3mf(parts: dict, path: str) -> None:
    """All parts in print orientation (lid upside down), side by side, as one 3MF."""
    plate = trimesh.Scene()
    x = 0.0
    for name, part in parts.items():
        s = part.copy()
        if name == "lid":
            s.rotate(180, axis=(1, 0, 0))
        s.min_to("z", 0).min_to("x", x).min_to("y", 0)
        plate.add_geometry(s.mesh, node_name=name, geom_name=name)
        x += s.xspan + 10
    plate.export(path)


def export_plate_3mf(parts: dict, path: str, bed=(350.0, 320.0), gap=10.0) -> None:
    """Lower box and tray one behind the other, lid turned 90 degrees beside
    them, centred on a bed of the given size (fits a Bambu Lab H2D)."""
    import numpy as np
    meshes = {}
    for name, part in parts.items():
        s = part.copy()
        if name == "lid":
            s.rotate(180, axis=(1, 0, 0)).rotate(90, axis=(0, 0, 1))
        s.min_to("x", 0).min_to("y", 0).min_to("z", 0)
        meshes[name] = s
    col_w = max(meshes["lower_box"].xspan, meshes["tray"].xspan)
    col_h = meshes["lower_box"].yspan + gap + meshes["tray"].yspan
    total_w = col_w + gap + meshes["lid"].xspan
    total_h = max(col_h, meshes["lid"].yspan)
    if total_w > bed[0] or total_h > bed[1]:
        print(f"warning: plate layout {total_w:.0f} x {total_h:.0f} exceeds the {bed[0]:.0f} x {bed[1]:.0f} bed")
    ox, oy = (bed[0] - total_w) / 2, (bed[1] - total_h) / 2
    meshes["lower_box"].translate((ox, oy, 0))
    meshes["tray"].translate((ox, oy + meshes["lower_box"].yspan + gap, 0))
    meshes["lid"].translate((ox + col_w + gap, oy, 0))
    plate = trimesh.Scene()
    for name, s in meshes.items():
        plate.add_geometry(s.mesh, node_name=name, geom_name=name)
    plate.export(path)


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "output_attenuator"))
