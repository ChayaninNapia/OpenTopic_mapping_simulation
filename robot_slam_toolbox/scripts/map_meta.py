import os
import yaml
from PIL import Image

# === CONFIG ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_obj_0v01_attemp3"
YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")

# === LOAD YAML METADATA ===
if not os.path.exists(YAML_PATH):
    raise FileNotFoundError(f"ไม่พบไฟล์ YAML: {YAML_PATH}")
with open(YAML_PATH, 'r') as f:
    meta = yaml.safe_load(f)

resolution = meta.get('resolution', None)
origin = meta.get('origin', None)
neg_thresh = meta.get('negate', None)
occ_thresh = meta.get('occupied_thresh', None)
free_thresh = meta.get('free_thresh', None)
mode = meta.get('mode', 'unknown')

# === LOAD PGM IMAGE ===
if not os.path.exists(PGM_PATH):
    raise FileNotFoundError(f"ไม่พบไฟล์ PGM: {PGM_PATH}")
img = Image.open(PGM_PATH)
width, height = img.size

# === DISPLAY ===
print("=== Map Metadata ===")
print(f"PGM path       : {PGM_PATH}")
print(f"PGM size       : {width} x {height} pixels")
print(f"Resolution     : {resolution} meters/pixel")
print(f"Origin         : {origin}")
print(f"Occupied thresh: {occ_thresh}")
print(f"Free thresh    : {free_thresh}")
print(f"Negate         : {neg_thresh}")
print(f"Mode           : {mode}")
print("====================")
