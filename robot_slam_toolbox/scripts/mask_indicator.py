#!/usr/bin/env python3
import os
import cv2
import numpy as np
from typing import Tuple
import yaml

# === CONFIGURATION ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
FILE_STEM = "ex1_std0001_attemp1"
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

    px, py = -4.0, 2.0
    pixelate_origin = [origin_x/resolution, origin_y/resolution]
    pixelate_p = [px/resolution, py/resolution]
    slam_x, slam_y = (-pixelate_origin[0]+pixelate_p[0]), (-pixelate_origin[1]+pixelate_p[1])

    side = 19  # ขนาดสี่เหลี่ยม (px)
    
    expansion_lv = 0
    expansion_factor = 2 * expansion_lv
    
    # สร้าง mask
    mask = create_square_mask((h, w), slam_x - expansion_lv, slam_y - expansion_lv, side + expansion_factor)

    # ใช้ mask ตรวจจับ occupy pixel
    occupied_threshold = 100  # สมมติว่า pixel สีเทาเข้ม/ดำ < 100 คือ occupied
    
    # masked_pgm = cv2.bitwise_and(pgm_img, pgm_img, mask=mask)
    
    # สร้างภาพพื้นหลังสีเทา (เช่น gray=180)
    gray_background = 180
    masked_pgm = np.full_like(pgm_img, gray_background)

    # วางค่า PGM เฉพาะจุดที่อยู่ใน mask (คือขอบสี่เหลี่ยม)
    masked_pgm[mask > 0] = pgm_img[mask > 0]

    # คำนวณ
    occupied_pixels = np.sum((masked_pgm < occupied_threshold) & (mask > 0))
    total_masked_pixels = np.sum(mask > 0)

    print(f"Occupied pixels inside square: {occupied_pixels}")
    print(f"Total pixels inside square   : {total_masked_pixels}")
    print(f"Occupancy ratio               : {occupied_pixels/total_masked_pixels:.2%}")
    
    # ทำภาพ copy ของ PGM เพื่อวาดทับ
    pgm_with_mask = cv2.cvtColor(pgm_img, cv2.COLOR_GRAY2BGR)  # แปลงเป็น BGR เพื่อวาดสี

    # วาดเส้นขอบสี (เช่น สีแดง) ที่ตำแหน่ง mask
    pgm_with_mask[mask > 0] = (0, 0, 255)  # สีแดงสด BGR (B=0, G=0, R=255)
    
    # === SAVE overlay image with red box (as separate file) ===
    # pgm_with_mask = pgm_img.copy()  # ✅ ยังเป็น grayscale อยู่
    # pgm_with_mask[mask > 0] = 0  # 0 = สีดำใน grayscale
    # overlay_save_path = os.path.join(MAPS_DIR, f"{FILE_STEM}_overlay.pgm")
    # cv2.imwrite(overlay_save_path, pgm_with_mask)
    # print(f"[INFO] Overlay image saved to: {overlay_save_path}")

    # แสดงเพื่อ debug
    cv2.imshow("PGM", pgm_img)
    cv2.imshow("PGM with Mask Overlay", pgm_with_mask)
    cv2.imshow(f'Mask level {expansion_lv}', mask)
    cv2.imshow("Masked PGM", masked_pgm)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
