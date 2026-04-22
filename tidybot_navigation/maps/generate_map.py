#!/usr/bin/env python3
"""
generate_map.py
===============
Generates a pre-built occupancy-grid map of the TidyBot home world that
exactly matches tidybot_home.world.

World coordinate system (metres):
  Room 1 interior: x=[0.00, 5.00]  y=[0.00, 4.00]
  Room 2 interior: x=[5.15, 9.15]  y=[0.00, 4.00]
  Shared wall    : x=[5.00, 5.15]
  Doorway gap    : y=[1.50, 2.50]  (1.00 m wide, wall x=[5.00,5.15])
  Outer walls    : 0.15 m thick (included in model geometry)

Map parameters (written to tidybot_home.yaml):
  resolution: 0.05 m/pixel
  origin    : (-0.30, -0.30, 0.0)  (map frame  =  world frame)
  occupied_thresh : 0.65
  free_thresh     : 0.25

PGM values:
  0   = occupied (black)
  205 = unknown  (grey, used outside mapped area)
  254 = free     (white)

Run:
  python3 generate_map.py
Outputs:
  tidybot_home.pgm
  tidybot_home.yaml
"""

import math
import os
import struct

# ── Map parameters ──────────────────────────────────────────────────────────
RESOLUTION   = 0.05          # m / pixel
ORIGIN_X     = -0.30         # world-x of map pixel (0,0) left edge
ORIGIN_Y     = -0.30         # world-y of map pixel (0,0) bottom edge
MAP_WIDTH_M  = 9.75          # total metres in x  (covers up to x=9.45)
MAP_HEIGHT_M = 4.60          # total metres in y  (covers up to y=4.30)

W = math.ceil(MAP_WIDTH_M  / RESOLUTION)   # pixels wide
H = math.ceil(MAP_HEIGHT_M / RESOLUTION)   # pixels tall
print(f"Map size: {W} x {H} pixels  ({W*RESOLUTION:.2f} x {H*RESOLUTION:.2f} m)")

OCCUPIED = 0
FREE     = 254
UNKNOWN  = 205


def world_to_pixel(wx, wy):
    """World (m) -> pixel col, row (row 0 = bottom in PGM convention)."""
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int((wy - ORIGIN_Y) / RESOLUTION)
    return col, row


def fill_rect_world(grid, wx0, wy0, wx1, wy1, value):
    """Fill a world-coordinate axis-aligned rectangle with <value>."""
    c0, r0 = world_to_pixel(wx0, wy0)
    c1, r1 = world_to_pixel(wx1, wy1)
    c0, c1 = sorted([c0, c1])
    r0, r1 = sorted([r0, r1])
    for r in range(max(0, r0), min(H, r1 + 1)):
        for c in range(max(0, c0), min(W, c1 + 1)):
            grid[r][c] = value


# ── Build grid (row 0 = y=ORIGIN_Y, row H-1 = top) ──────────────────────────
# Initialise everything as UNKNOWN
grid = [[UNKNOWN] * W for _ in range(H)]

# --- Free space: Room 1 interior -------------------------------------------
fill_rect_world(grid, 0.00, 0.00, 5.00, 4.00, FREE)

# --- Free space: Room 2 interior -------------------------------------------
fill_rect_world(grid, 5.15, 0.00, 9.15, 4.00, FREE)

# --- Free space: doorway gap (x=[5.00,5.15], y=[1.50,2.50]) ----------------
fill_rect_world(grid, 5.00, 1.50, 5.15, 2.50, FREE)

# --- Outer walls (OCCUPIED) -------------------------------------------------
# West wall:  x=[-0.15, 0.00]
fill_rect_world(grid, -0.15, -0.15, 0.00, 4.15, OCCUPIED)
# South wall: y=[-0.15, 0.00]
fill_rect_world(grid, -0.15, -0.15, 9.30, 0.00, OCCUPIED)
# North wall: y=[4.00, 4.15]
fill_rect_world(grid, -0.15, 4.00, 9.30, 4.15, OCCUPIED)
# East wall:  x=[9.15, 9.30]
fill_rect_world(grid, 9.15, -0.15, 9.30, 4.15, OCCUPIED)

# --- Shared wall (south segment: y=[0,1.5]) ---------------------------------
fill_rect_world(grid, 5.00, 0.00, 5.15, 1.50, OCCUPIED)

# --- Shared wall (north segment: y=[2.5,4.0]) --------------------------------
fill_rect_world(grid, 5.00, 2.50, 5.15, 4.00, OCCUPIED)

# --- Furniture obstacles (Room 1) -------------------------------------------
# Sofa: 1.8 x 0.7 centred at (1.0, 0.45)
fill_rect_world(grid, 0.10, 0.10, 1.90, 0.80, OCCUPIED)
# Coffee table: 0.8 x 0.5 centred at (1.0, 1.35)
fill_rect_world(grid, 0.60, 1.10, 1.40, 1.60, OCCUPIED)
# Dining table: 1.2 x 0.8 centred at (3.8, 2.0)
fill_rect_world(grid, 3.20, 1.60, 4.40, 2.40, OCCUPIED)
# Chair 1: (3.5, 1.3)  0.45 x 0.45
fill_rect_world(grid, 3.27, 1.07, 3.72, 1.52, OCCUPIED)
# Chair 2: (4.1, 2.8)  0.45 x 0.45
fill_rect_world(grid, 3.87, 2.57, 4.32, 3.02, OCCUPIED)
# Bookshelf: 0.8 x 0.30 centred at (0.5, 3.87)
fill_rect_world(grid, 0.10, 3.72, 0.90, 4.02, OCCUPIED)

# --- Furniture obstacles (Room 2) -------------------------------------------
# Kitchen counter: 2.0 x 0.6 centred at (7.15, 0.35)
fill_rect_world(grid, 6.15, 0.05, 8.15, 0.65, OCCUPIED)
# Kitchen table: 1.0 x 0.8 centred at (7.15, 2.5)
fill_rect_world(grid, 6.65, 2.10, 7.65, 2.90, OCCUPIED)
# Chair 3: (6.5, 2.5)  0.45 x 0.45
fill_rect_world(grid, 6.27, 2.27, 6.72, 2.72, OCCUPIED)
# Chair 4: (7.8, 3.2)  0.45 x 0.45
fill_rect_world(grid, 7.57, 2.97, 8.02, 3.42, OCCUPIED)
# Cabinet: 0.6 x 0.35 centred at (9.07, 2.0)
fill_rect_world(grid, 8.77, 1.82, 9.37, 2.17, OCCUPIED)

# ── Write PGM (P5 binary, row 0 = TOP in PGM = our max-y row) ────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
pgm_path   = os.path.join(script_dir, "tidybot_home.pgm")
yaml_path  = os.path.join(script_dir, "tidybot_home.yaml")

with open(pgm_path, "wb") as f:
    header = f"P5\n{W} {H}\n255\n"
    f.write(header.encode("ascii"))
    # PGM row 0 = image top = world max-y
    for row in reversed(range(H)):
        f.write(bytes(grid[row]))

print(f"Written: {pgm_path}")

# ── Write YAML ───────────────────────────────────────────────────────────────
yaml_content = f"""\
image: tidybot_home.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
"""
with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"Written: {yaml_path}")
print("Map generation complete.")
