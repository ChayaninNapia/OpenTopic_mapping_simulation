#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.ndimage import distance_transform_cdt

def psi(source_map, target_dmap, c=1):
    points = np.argwhere(source_map == c)
    distances = [target_dmap[y, x] for y, x in points]
    return np.mean(distances) if distances else 0

def compute_metrics(gt, target):
    dmap_gt     = distance_transform_cdt(1 - gt,     metric='taxicab')
    dmap_target = distance_transform_cdt(1 - target, metric='taxicab')
    psi_1  = psi(target,    dmap_gt)     # ψ(a' → a)
    psi_2  = psi(gt,        dmap_target) # ψ(a → a')
    return psi_1, psi_2, psi_1 + psi_2, dmap_gt, dmap_target

def load_and_binarize(path, thresh=250):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"ไม่พบไฟล์: {path}")
    return (img < thresh).astype(np.uint8)

def show_maps_4panel(gt, target, dmap_gt, dmap_target, title_suffix=""):
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes[0,0].imshow(gt,         cmap='gray');    axes[0,0].set_title("GT Binary")
    axes[0,1].imshow(target,     cmap='gray');    axes[0,1].set_title(f"Target Binary{title_suffix}")
    axes[1,0].imshow(dmap_gt,     cmap='inferno'); axes[1,0].set_title("D-map GT")
    axes[1,1].imshow(dmap_target, cmap='inferno'); axes[1,1].set_title(f"D-map Target{title_suffix}")
    for ax in axes.flatten():
        ax.axis('off')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    MAPS_DIR        = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"  # ปรับ path ให้ตรง
    GT_STEM         = "origin_map"
    ATTEMPT_PREFIX  = "ex2_std0010_attemp"
    NUM_ATTEMPTS    = 8

    # โหลด GT แค่ครั้งเดียว
    gt_path = os.path.join(MAPS_DIR, f"{GT_STEM}.pgm")
    gt = load_and_binarize(gt_path)

    results = []
    for i in range(1, NUM_ATTEMPTS + 1):
        stem = f"{ATTEMPT_PREFIX}{i}"
        tgt_path = os.path.join(MAPS_DIR, f"{stem}.pgm")
        try:
            target = load_and_binarize(tgt_path)
        except FileNotFoundError as e:
            print(e)
            continue

        # ถ้าต้องการ crop ให้ขนาดเท่ากัน uncomment ข้างล่างนี้
        h = min(gt.shape[0], target.shape[0])
        w = min(gt.shape[1], target.shape[1])
        gt_c   = gt[:h, :w]
        tgt_c  = target[:h, :w]
        psi_1, psi_2, psi_tot, d_gt, d_tgt = compute_metrics(gt_c, tgt_c)

        # psi_1, psi_2, psi_tot, d_gt, d_tgt = compute_metrics(gt, target)
        print(f"--- Attempt {i} ---")
        print(f"ψ(target→GT): {psi_1:.5f}")
        print(f"ψ(GT→target): {psi_2:.5f}")
        print(f"Ψ total      : {psi_tot:.5f}\n")

        # เก็บผลลัพธ์ไว้ใช้ต่อ
        results.append({
            "attempt": i,
            "psi_1": psi_1,
            "psi_2": psi_2,
            "psi_total": psi_tot
        })

        # ถ้าอยากดูภาพแต่ละรอบ เปิดฟังก์ชันนี้
        # show_maps_4panel(gt, target, d_gt, d_tgt, title_suffix=f" #{i}")

    # สรุปผลรวบยอด
    print("Summary of all attempts:")
    for r in results:
        print(f"  Att {r['attempt']}: Ψ={r['psi_total']:.5f}")

    # จะเพิ่มโค้ดเพื่อเซฟเป็น CSV, plot หรืออื่นๆ ก็ทำต่อได้สบายๆ
