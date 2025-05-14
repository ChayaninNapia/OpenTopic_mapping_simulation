#!/usr/bin/env python3
import os
import cv2
import yaml
import matplotlib.pyplot as plt
from math import hypot
from matplotlib.widgets import Button

# === CONFIGURATION ===
MAPS_DIR  = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_std0001_attemp1"
YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
FILE_STEM = "origin_map_overlay"
PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")

# === LOAD YAML METADATA ===
with open(YAML_PATH, 'r') as f:
    meta = yaml.safe_load(f)
resolution = float(meta["resolution"])  # meters per pixel

# === LOAD IMAGE ===
img = cv2.imread(PGM_PATH, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Image not found: {PGM_PATH}")
if img.ndim == 3:
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# === STATE VARIABLES ===
points = []
measure_mode = False  # only record clicks when True

# === SETUP FIGURE & BUTTONS ===
fig, ax = plt.subplots(figsize=(8,8))
plt.subplots_adjust(bottom=0.15)
ax.imshow(img, cmap='gray')
ax.set_title("Press ‘Measure’ to start picking two points")

# button axes
ax_reset   = plt.axes([0.2, 0.05, 0.2, 0.05])
ax_measure = plt.axes([0.6, 0.05, 0.2, 0.05])
btn_reset   = Button(ax_reset,   "Reset")
btn_measure = Button(ax_measure, "Measure")

# === CALLBACKS ===
def onclick(event):
    global measure_mode, points
    if not measure_mode:
        return  # ignore clicks until Measure pressed
    if event.inaxes != ax:
        return
    if len(points) < 2:
        points.append((event.xdata, event.ydata))
        ax.plot(event.xdata, event.ydata, 'r+', markersize=12)
        fig.canvas.draw()
        print(f"  Point {len(points)} at ({event.xdata:.1f}, {event.ydata:.1f})")
    if len(points) == 2:
        # compute & display
        (x0, y0), (x1, y1) = points
        pix    = hypot(x1 - x0, y1 - y0)
        meters = pix * resolution
        ax.plot([x0, x1], [y0, y1], 'r-')
        fig.canvas.draw()
        print("\n--- Measurement Result ---")
        print(f"Pixel distance : {pix:.2f} px")
        print(f"Resolution     : {resolution:.3f} m/px")
        print(f"Real distance  : {meters:.3f} m\n")
        # exit measure mode automatically
        measure_mode = False
        ax.set_title("Done. Press ‘Measure’ to pick new points")

def do_measure(event):
    global measure_mode, points
    # reset any old picks
    points = []
    ax.clear()
    ax.imshow(img, cmap='gray')
    ax.set_title("Click TWO points now")
    print("--- Entering measurement mode ---")
    measure_mode = True
    fig.canvas.draw()

def do_reset(event):
    global measure_mode, points
    measure_mode = False
    points = []
    ax.clear()
    ax.imshow(img, cmap='gray')
    ax.set_title("Press ‘Measure’ to start picking two points")
    print("All cleared. Press Measure when ready.")
    fig.canvas.draw()

# === CONNECT EVENTS ===
fig.canvas.mpl_connect('button_press_event', onclick)
btn_measure.on_clicked(do_measure)
btn_reset.on_clicked(do_reset)

plt.show()
