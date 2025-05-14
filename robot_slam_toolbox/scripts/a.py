#!/usr/bin/env python3
import os
import cv2
import numpy as np
from typing import Tuple
import yaml

# === CONFIGURATION ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_obj_0v01_attemp1"
YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")

# === LOAD YAML METADATA ===
with open(YAML_PATH, 'r') as f:
    meta = yaml.safe_load(f)

resolution = meta["resolution"]                     # eg. 0.05
origin_x, origin_y, *_ = meta["origin"]    

def slam_to_opencv(px_slam: float, py_slam: float, img_height: int) -> Tuple[int, int]:
    px_cv = int(round(px_slam))
    py_cv = int(round(img_height - 1 - py_slam))
    return px_cv, py_cv

def create_square_mask(
    img_shape: Tuple[int, int],
    px_slam: float, py_slam: float,
    side_len: int
) -> np.ndarray:
    """
    สร้าง mask ภาพขนาดเท่า img_shape
    มีแต่สี่เหลี่ยมขนาด side_len x side_len ที่ตำแหน่งกำหนด
    """
    h, w = img_shape
    mask = np.zeros((h, w), dtype=np.uint8)  # สร้างภาพดำล้วน

    bl_x, bl_y = slam_to_opencv(px_slam, py_slam, h)
    tr_x, tr_y = slam_to_opencv(px_slam + side_len, py_slam + side_len, h)

    top_left = (bl_x, tr_y)
    bottom_right = (tr_x, bl_y)

    # วาดสี่เหลี่ยมขาว (255) ลงใน mask
    cv2.rectangle(mask, top_left, bottom_right, 255, thickness=1)

    return mask

# ── ตัวอย่างใช้งาน ──
if __name__ == "__main__":
    pgm_img = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
    h, w = pgm_img.shape

    # แปลงพิกัด SLAM → pixel
    px, py = -0.5, 2.0
    pixelate_origin = [origin_x / resolution, origin_y / resolution]
    pixelate_p = [px / resolution, py / resolution]
    slam_x = -pixelate_origin[0] + pixelate_p[0]
    slam_y = -pixelate_origin[1] + pixelate_p[1]

    side = 20  # ขนาดสี่เหลี่ยม (px)
    occupied_threshold = 100  # <100 คือ occupied

    # ลูปไล่ค่า expansion_lv
    for expansion_lv in range(0, 5):  # 0 ถึง 4
        expansion_factor = 2 * expansion_lv

        mask = create_square_mask(
            (h, w),
            slam_x - expansion_lv,
            slam_y - expansion_lv,
            side + expansion_factor
        )

        masked_area = mask > 0
        pgm_values = pgm_img[masked_area]

        occupied_pixels = np.sum(pgm_values < occupied_threshold)
        black_pixels = np.sum(pgm_values == 0)
        total_masked_pixels = np.sum(masked_area)

        ratio_occupied = (occupied_pixels / total_masked_pixels) if total_masked_pixels else 0
        ratio_black = (black_pixels / total_masked_pixels) if total_masked_pixels else 0

        print(f"[LV {expansion_lv:2d}]")
        print(f"   Occupied pixels (<{occupied_threshold}): {occupied_pixels:5d} ({ratio_occupied:.2%})")
        print(f"   Black pixels     (==0)                : {black_pixels:5d} ({ratio_black:.2%})")
        print(f"   Total pixels inside mask             : {total_masked_pixels:5d}\n")

