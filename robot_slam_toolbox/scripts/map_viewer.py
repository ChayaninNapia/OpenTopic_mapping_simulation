#!/usr/bin/env python3
import os
import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# === CONFIGURATION ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_std0001_attemp1"
YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
FILE_STEM = "origin_map_overlay"
PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")

# Load metadata and image
with open(YAML_PATH, 'r') as f:
    meta = yaml.safe_load(f)
resolution = meta['resolution']  # meters/pixel

data = np.array(Image.open(PGM_PATH))

# Create figure and axis
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
ax.imshow(data, cmap='gray')
ax.set_title('Map Measure (use Measure/Reset buttons)')

# State variables
draw_artists = []
points = []
measuring = False

def onclick(event):
    global measuring, points
    if not measuring or event.inaxes != ax:
        return

    x, y = event.xdata, event.ydata
    dot, = ax.plot(x, y, 'ro')
    draw_artists.append(dot)
    points.append((x, y))
    fig.canvas.draw()

    if len(points) == 2:
        (x1, y1), (x2, y2) = points
        line, = ax.plot([x1, x2], [y1, y2], 'r-')
        draw_artists.append(line)

        # คำนวณระยะ
        d_px = np.hypot(x2 - x1, y2 - y1)      # float pixel distance
        int_px = int(d_px)                    # integer pixel
        frac_px = d_px - int_px               # fractional pixel
        d_interp_m = d_px * resolution        # ระยะจริงแบบ interpolate
        d_int_only_m = int_px * resolution    # ระยะจริงเอา integer เท่านั้น

        # ปริ้นผล
        print(f"Pixel distance: {d_px:.2f} px")
        print(f"  → {int_px} px + {frac_px:.2f} px")
        print(f"Real-world distance (interpolated): {d_interp_m:.3f} m")
        print(f"Real-world distance (integer-only):  {d_int_only_m:.3f} m")

        measuring = False
        btn_measure.label.set_text('Measure')

def activate_measure(event):
    global measuring, points
    measuring = True
    points = []
    btn_measure.label.set_text('Measuring...')

def reset_points(event):
    global points, draw_artists, measuring
    for art in draw_artists:
        try:
            art.remove()
        except:
            pass
    draw_artists.clear()
    points = []
    measuring = False
    btn_measure.label.set_text('Measure')
    fig.canvas.draw()

# สร้างปุ่ม
ax_measure = plt.axes([0.65, 0.05, 0.1, 0.075])
btn_measure = Button(ax_measure, 'Measure')
btn_measure.on_clicked(activate_measure)

ax_reset = plt.axes([0.77, 0.05, 0.1, 0.075])
btn_reset = Button(ax_reset, 'Reset')
btn_reset.on_clicked(reset_points)

# เชื่อมอีเวนต์
cid_click = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
