import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.ndimage import distance_transform_cdt


def psi(source_map, target_dmap, c=1):
    points = np.argwhere(source_map == c)
    distances = [target_dmap[y, x] for y, x in points]
    return np.mean(distances) if distances else 0

def debug(gt, target, dmap_gt, psi_1, psi_2, psi_total):
    print("target == gt:", np.array_equal(target, gt))  # ต้องได้ True
    print("np.unique(gt):", np.unique(gt))
    print("np.unique(target):", np.unique(target))
    print("np.unique(dmap_gt):", np.unique(dmap_gt))

    print(f"ψ(map_gen → GT): {psi_1}")
    print(f"ψ(GT → map_gen): {psi_2}")
    print(f"Ψ total (ψ both ways): {psi_total}")
    
def show_maps_4panel(gt, target, dmap_gt, dmap_target):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # รูป 1: Ground Truth (binary)
    im1 = axes[0, 0].imshow(gt.astype(np.float32), cmap='gray')
    axes[0, 0].set_title("GT Binary")
    plt.colorbar(im1, ax=axes[0, 0])

    # รูป 2: Target (binary)
    im2 = axes[0, 1].imshow(target.astype(np.float32), cmap='gray')
    axes[0, 1].set_title("Target Binary")
    plt.colorbar(im2, ax=axes[0, 1])

    # รูป 3: D-map GT
    im3 = axes[1, 0].imshow(dmap_gt, cmap='inferno')
    axes[1, 0].set_title("D-map GT")
    plt.colorbar(im3, ax=axes[1, 0])

    # รูป 4: D-map Target
    im4 = axes[1, 1].imshow(dmap_target, cmap='inferno')
    axes[1, 1].set_title("D-map Target")
    plt.colorbar(im4, ax=axes[1, 1])

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xticks([])
            ax.set_yticks([])

    plt.tight_layout()
    plt.show()

# === CONFIGURATION ===
MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
GT_FILE_STEM = "origin_map"
TARGET_FILE_STEM = "ex3_encoder_std0005_attemp2"
GT_PGM_PATH  = os.path.join(MAPS_DIR, f"{GT_FILE_STEM}.pgm")
TARGET_PGM_PATH = os.path.join(MAPS_DIR, f"{TARGET_FILE_STEM}.pgm")

# โหลด PGM
gt_img = cv2.imread(GT_PGM_PATH, cv2.IMREAD_UNCHANGED)
target_img = cv2.imread(TARGET_PGM_PATH, cv2.IMREAD_UNCHANGED)

example_gt = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
])

example_target = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1],
])


# แปลงเป็น binary: 1 = occupied, 0 = free
# (เช่น ถ้า 0=สิ่งกีดขวาง → pixel เข้ม)
# แปลงเป็น binary map
# สมมุติว่า 0 = occupied (seed), 1 = free

gt = (gt_img < 250).astype(np.uint8)
target = (target_img < 250).astype(np.uint8)

# crop map ทั้งสองให้เท่ากัน
min_h = min(gt.shape[0], target.shape[0])
min_w = min(gt.shape[1], target.shape[1])

# gt = gt[:min_h, :min_w]
# target = target[:min_h, :min_w]

offset = [0, 0]

gt = gt[offset[1]:min_h-offset[1], offset[0]:min_w-offset[0]]
target = target[offset[1]:min_h-offset[1], offset[0]:min_w-offset[0]]

# คำนวณ distance map (taxicab = Manhattan)
dmap_ex_gt= distance_transform_cdt(1 - example_gt, metric='taxicab')
dmap_ex_target = distance_transform_cdt(1 - example_target, metric='taxicab')

# ######### EXAMPLE ##########
# ex_psi_1 = psi(example_target, dmap_ex_gt)   # ψ(a, a′)
# ex_psi_2 = psi(example_gt, dmap_ex_target)  # ψ(a′, a)
# ex_psi_total = ex_psi_1 + ex_psi_2
# debug(example_gt, example_target, dmap_ex_gt, ex_psi_1, ex_psi_2, ex_psi_total)
# show_maps_4panel(example_gt, example_target, dmap_ex_gt, dmap_ex_target)

######### IMPLEMENT ##########
dmap_gt = distance_transform_cdt(1 - gt, metric='taxicab')  # 1-binary เพราะมันหาจาก free → occupied
dmap_target = distance_transform_cdt(1 - target, metric='taxicab') 

psi_1 = psi(target, dmap_gt)   # ψ(a, a′)
psi_2 = psi(gt, dmap_target)  # ψ(a′, a)
psi_total = psi_1 + psi_2
debug(gt, target, dmap_gt, psi_1, psi_2, psi_total)
show_maps_4panel(gt, target, dmap_gt, dmap_target)

# fig, ax = plt.subplots(figsize=(6, 6))
# cax = ax.imshow(dmap_ex_gt, cmap='inferno')
# fig.colorbar(cax, ax=ax)
# plt.title("D-map using distance_transform_cdt")
# plt.axis('off')
# plt.tight_layout()
# plt.show()

# print("pgm unique:", np.unique(img))
# print("binary unique:", np.unique(binary))
# print("dmap min/max:", dmap.min(), dmap.max())


