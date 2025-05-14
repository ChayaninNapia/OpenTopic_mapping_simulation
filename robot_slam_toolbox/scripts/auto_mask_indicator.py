#!/usr/bin/env python3
import os
import cv2
import numpy as np
from typing import Tuple
import yaml

# === CONFIGURATION ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_obj_0v02_attemp3"
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

    side = 19
    occupied_threshold = 100

    black_pixel_list = []
    occupied_pixel_list = []
    result_by_lv = []

    # Loop expansion lv
    for expansion_lv in range(0, 5):
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

        occupied_pixel_list.append(occupied_pixels)
        black_pixel_list.append(black_pixels)
        result_by_lv.append({
            "lv": expansion_lv,
            "occupied": occupied_pixels,
            "black": black_pixels,
            "total": total_masked_pixels
        })

    # รวมยอด
    total_black_all_lv = sum(black_pixel_list)
    total_occupied_all_lv = sum(occupied_pixel_list)

    # แสดงผล per level
    for entry in result_by_lv:
        lv = entry["lv"]
        occ = entry["occupied"]
        blk = entry["black"]
        total = entry["total"]
        ratio_occ = occ / total if total else 0
        ratio_blk = blk / total if total else 0
        percent_blk_total = blk / total_black_all_lv if total_black_all_lv else 0

        print(f"[LV {lv:2d}]")
        print(f"   Occupied pixels (<{occupied_threshold}): {occ:5d} ({ratio_occ:.2%})")
        print(f"   Black pixels     (==0)                : {blk:5d} ({ratio_blk:.2%})")
        print(f"   Share of total black pixels           : {percent_blk_total:.2%}")
        print(f"   Total pixels inside mask             : {total:5d}\n")

    # แสดงผลรวมท้าย
    print("═" * 50)
    print(f"💥 Total Occupied pixels (all levels): {total_occupied_all_lv}")
    print(f"🖤 Total Black pixels    (all levels): {total_black_all_lv}")
    print("═" * 50)


