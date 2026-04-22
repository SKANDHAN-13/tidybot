#!/usr/bin/env python3
"""
tools/compute_inertia.py
========================
Analytical inertia tensor calculator for the TidyBot URDF links.

Implements the standard rigid-body formulas for solid cuboids and solid
cylinders (the only primitives used in tidybot.urdf.xacro), then prints a
URDF-ready <inertia> block for each link.

Usage
-----
    python3 tools/compute_inertia.py

The printed values were used to fill the xacro macros in:
    tidybot_description/urdf/tidybot.urdf.xacro

    box_inertia (m, w, d, h):
        ixx = m/12 * (d² + h²)
        iyy = m/12 * (w² + h²)
        izz = m/12 * (w² + d²)

    cyl_inertia_y (m, r, l)  — cylinder axis along Y (wheels):
        ixx = izz = m/12 * (3r² + l²)
        iyy = m/2  * r²

    cyl_inertia_z (m, r, l)  — cylinder axis along Z (lidar):
        ixx = iyy = m/12 * (3r² + l²)
        izz = m/2  * r²

References
----------
Beer & Johnston, "Vector Mechanics for Engineers", Table 9.2
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Literal


# ── Data classes ─────────────────────────────────────────────────────────────

@dataclass
class Inertia:
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def urdf_block(self, indent: int = 8) -> str:
        pad = " " * indent
        return (
            f'{pad}<inertia\n'
            f'{pad}  ixx="{self.ixx:.8g}" ixy="{self.ixy:.8g}" ixz="{self.ixz:.8g}"\n'
            f'{pad}  iyy="{self.iyy:.8g}" iyz="{self.iyz:.8g}"\n'
            f'{pad}  izz="{self.izz:.8g}"/>'
        )


@dataclass
class Link:
    name: str
    mass: float
    shape: Literal["box", "cyl_y", "cyl_z"]
    # box  → (w, d, h)   metres
    # cyl  → (r, l)      metres
    dims: tuple

    def inertia(self) -> Inertia:
        m = self.mass
        if self.shape == "box":
            w, d, h = self.dims
            return Inertia(
                ixx=m / 12 * (d**2 + h**2),
                iyy=m / 12 * (w**2 + h**2),
                izz=m / 12 * (w**2 + d**2),
            )
        elif self.shape == "cyl_y":          # wheel axis = Y
            r, l = self.dims
            it = m / 12 * (3 * r**2 + l**2)
            ia = m / 2  * r**2
            return Inertia(ixx=it, iyy=ia, izz=it)
        elif self.shape == "cyl_z":          # lidar axis = Z
            r, l = self.dims
            it = m / 12 * (3 * r**2 + l**2)
            ia = m / 2  * r**2
            return Inertia(ixx=it, iyy=it, izz=ia)
        else:
            raise ValueError(f"Unknown shape: {self.shape}")


# ── TidyBot link definitions (from URDF comments) ────────────────────────────
#   base_link        : box  0.50 x 0.50 x 0.12 m   mass 20 kg
#   wheel (x4)       : cyl  r=0.05  h=0.05          mass  1 kg  (axis y)
#   torso_link       : box  0.25 x 0.20 x 0.45 m   mass  5 kg
#   right upper arm  : box  0.05 x 0.05 x 0.28 m   mass  0.50 kg
#   right lower arm  : box  0.05 x 0.05 x 0.23 m   mass  0.40 kg
#   right wrist      : box  0.07 x 0.07 x 0.07 m   mass  0.15 kg
#   right wrist conn : box  0.06 x 0.06 x 0.05 m   mass  0.10 kg
#   gripper base     : box  0.10 x 0.05 x 0.05 m   mass  0.15 kg
#   gripper finger×2 : box  0.01 x 0.015 x 0.08 m  mass  0.05 kg
#   head_link        : box  0.26 x 0.05 x 0.18 m   mass  0.50 kg
#   camera_link      : box  0.07 x 0.07 x 0.04 m   mass  0.10 kg
#   lidar_link       : cyl  r=0.05  h=0.04          mass  0.30 kg  (axis z)
#   imu_link         : box  0.04 x 0.04 x 0.04 m   mass  0.05 kg
#   contact_link     : box  0.08 x 0.04 x 0.02 m   mass  0.05 kg
#   ultrasonic_link  : box  0.04 x 0.04 x 0.02 m   mass  0.05 kg
#   left upper arm   : box  0.05 x 0.05 x 0.28 m   mass  0.50 kg  (mirror)
#   left lower arm   : box  0.05 x 0.05 x 0.23 m   mass  0.40 kg  (mirror)
#   left wrist       : box  0.07 x 0.07 x 0.07 m   mass  0.15 kg  (mirror)

LINKS: list[Link] = [
    Link("base_link",              20.00, "box",   (0.50, 0.50, 0.12)),
    Link("wheel (each, x4)",        1.00, "cyl_y", (0.05, 0.05)),
    Link("torso_link",              5.00, "box",   (0.25, 0.20, 0.45)),
    Link("right_upper_arm_link",    0.50, "box",   (0.05, 0.05, 0.28)),
    Link("right_lower_arm_link",    0.40, "box",   (0.05, 0.05, 0.23)),
    Link("right_wrist_link",        0.15, "box",   (0.07, 0.07, 0.07)),
    Link("right_wrist_connector",   0.10, "box",   (0.06, 0.06, 0.05)),
    Link("right_gripper_base",      0.15, "box",   (0.10, 0.05, 0.05)),
    Link("right_finger (each, x2)", 0.05, "box",   (0.01, 0.015, 0.08)),
    Link("head_link",               0.50, "box",   (0.26, 0.05, 0.18)),
    Link("camera_link",             0.10, "box",   (0.07, 0.07, 0.04)),
    Link("lidar_link",              0.30, "cyl_z", (0.05, 0.04)),
    Link("imu_link",                0.05, "box",   (0.04, 0.04, 0.04)),
    Link("contact_sensor_link",     0.05, "box",   (0.08, 0.04, 0.02)),
    Link("ultrasonic_link (each)",  0.05, "box",   (0.04, 0.04, 0.02)),
    Link("left_upper_arm_link",     0.50, "box",   (0.05, 0.05, 0.28)),
    Link("left_lower_arm_link",     0.40, "box",   (0.05, 0.05, 0.23)),
    Link("left_wrist_link",         0.15, "box",   (0.07, 0.07, 0.07)),
]


def total_mass() -> float:
    # 4 wheels + 3 ultrasonic sensors
    extra = 3 * 1.00 + 2 * 0.05 + 1 * 0.05   # 3 extra wheels + 2 extra fingers + l/r ultrasonics
    return sum(lk.mass for lk in LINKS) + extra


def print_table() -> None:
    col = 38
    print("=" * 80)
    print("TidyBot — Inertia Tensor Summary")
    print("=" * 80)
    print(f"{'Link':{col}} {'mass (kg)':>10}  {'ixx':>12}  {'iyy':>12}  {'izz':>12}")
    print("-" * 80)
    for lk in LINKS:
        I = lk.inertia()
        print(f"{lk.name:{col}} {lk.mass:>10.3f}  {I.ixx:>12.6f}  {I.iyy:>12.6f}  {I.izz:>12.6f}")
    print("-" * 80)
    print(f"\nTotal robot mass (approx): {total_mass():.2f} kg")
    print()


def print_urdf_blocks() -> None:
    print("=" * 80)
    print("URDF <inertial> blocks")
    print("=" * 80)
    for lk in LINKS:
        I = lk.inertia()
        print(f"\n<!-- {lk.name}  (mass={lk.mass} kg, shape={lk.shape}, dims={lk.dims}) -->")
        print(f"        <inertial>")
        print(f"          <mass value=\"{lk.mass}\"/>")
        print(I.urdf_block(indent=10))
        print(f"        </inertial>")


def plot_geometry() -> None:
    """Optional: draw a schematic of each link geometry (requires matplotlib)."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import numpy as np
    except ImportError:
        print("matplotlib not installed — skipping geometry plot.")
        return

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.set_title("TidyBot link geometry (side-view bounding boxes, not to scale)")
    ax.set_aspect("equal")
    ax.axis("off")

    x_cursor = 0.05
    colours = plt.cm.tab20.colors
    for i, lk in enumerate(LINKS):
        if lk.shape == "box":
            w, d, h = lk.dims
        else:
            r, l = lk.dims
            w = 2 * r
            h = l
        rect = mpatches.FancyBboxPatch(
            (x_cursor, 0), w * 5, h * 5,
            boxstyle="round,pad=0.01",
            linewidth=1,
            edgecolor="black",
            facecolor=colours[i % len(colours)],
            alpha=0.7,
        )
        ax.add_patch(rect)
        ax.text(
            x_cursor + w * 5 / 2, h * 5 + 0.05,
            lk.name.replace("_link", "").replace("right_", "R.").replace("left_", "L."),
            fontsize=6, ha="center", rotation=45,
        )
        x_cursor += w * 5 + 0.08

    ax.set_xlim(0, x_cursor + 0.1)
    ax.set_ylim(-0.2, 2.5)
    plt.tight_layout()
    out = "tidybot_link_geometry.png"
    plt.savefig(out, dpi=150)
    print(f"Geometry plot written to {out}")
    plt.show()


if __name__ == "__main__":
    print_table()
    print_urdf_blocks()
    plot_geometry()
