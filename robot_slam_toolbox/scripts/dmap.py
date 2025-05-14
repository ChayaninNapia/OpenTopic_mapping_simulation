import os
import cv2
import numpy as np
from typing import Tuple
import yaml


INF = 10**6  # ค่าที่แทน ∞ ต้องมากกว่า max possible distance

def load_occupancy_map(pgm_path: str, yaml_path: str, occupied_thresh: float = 0.5) -> np.ndarray:
    """
    โหลดภาพ pgm + config yaml แล้วคืน binary map (1 = occupied, 0 = free)
    """
    # อ่าน YAML เพื่อดึงข้อมูล threshold (ถ้ามี)
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)
    # สมมติใน YAML มี key 'occupied_thresh' (0.0-1.0)
    occupied_thresh = cfg.get('occupied_thresh', occupied_thresh)
    
    # โหลด PGM (0-255)
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Cannot load PGM: {pgm_path}")
    
    # แปลงเป็นค่า 0-1 ตาม threshold
    norm = img.astype(np.float32) / 255.0
    binary = (norm >= occupied_thresh).astype(np.uint8)
    return binary

def compute_d_map_manual(binary_map: np.ndarray, c: int = 1) -> np.ndarray:
    """
    คำนวณ d_map แบบ manual two-pass relaxation (Manhattan distance)
      - binary_map: 2D array (0=free, 1=occupied)
      - c: ค่าใน binary_map ที่ถือเป็น seed (default: occupied=1)
    คืนค่า d_map (same shape) ที่ d_map[y][x] = Manhattan distance to nearest seed
    """
    n, m = binary_map.shape
    # 1) Initialization
    d_map = np.full((n, m), INF, dtype=np.int32)
    # จุด seed = 0
    seeds = (binary_map == c)
    d_map[seeds] = 0

    # 2) Relaxation pass 1: scan top-left → bottom-right
    for y in range(n):
        for x in range(m):
            # check left
            if x > 0:
                d_map[y, x] = min(d_map[y, x], d_map[y, x-1] + 1)
            # check up
            if y > 0:
                d_map[y, x] = min(d_map[y, x], d_map[y-1, x] + 1)

    # 3) Relaxation pass 2: scan bottom-right → top-left
    for y in range(n-1, -1, -1):
        for x in range(m-1, -1, -1):
            # check right
            if x < m-1:
                d_map[y, x] = min(d_map[y, x], d_map[y, x+1] + 1)
            # check down
            if y < n-1:
                d_map[y, x] = min(d_map[y, x], d_map[y+1, x] + 1)

    return d_map

def main():
    
    # === CONFIGURATION ===
    MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
    FILE_STEM = "ex1_obj_0v01_attemp1"
    YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
    PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")

    
    # โหลด binary occupancy map
    occ = load_occupancy_map(PGM_PATH, YAML_PATH)
    print(f"Loaded binary map: shape={occ.shape}, occupied={(occ==1).sum()} pts")

    # คำนวณ d_map โดยไม่ใช้ SciPy
    dmap = compute_d_map_manual(occ, c=1)
    print("d_map computed. Sample values:")
    print(dmap)

    # (ถ้าต้องการ) บันทึก d_map เป็นไฟล์ .npy หรือรูปภาพ
    np.save(os.path.join(MAPS_DIR, f"{FILE_STEM}_dmap.npy"), dmap)
    # หรือ normalize แล้ว save as PNG for quick check
    cv2.imwrite(os.path.join(MAPS_DIR, f"{FILE_STEM}_dmap.png"),
                (255 * (dmap.astype(np.float32) / dmap.max())).astype(np.uint8))

if __name__ == "__main__":
    main()
