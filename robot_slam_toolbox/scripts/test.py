import cv2
from typing import Tuple
import os

def slam_to_opencv(px_slam: float, py_slam: float, img_height: int) -> Tuple[int, int]:
    """
    แปลงพิกัดพิกเซลจาก SLAM-Toolbox (origin มุมซ้ายล่าง)
    เป็นพิกัด OpenCV (origin มุมซ้ายบน)
    """
    px_cv = int(round(px_slam))
    py_cv = int(round(img_height - 1 - py_slam))
    return px_cv, py_cv

def draw_square(
    img: cv2.UMat, 
    px_slam: int, py_slam: int, 
    side_len: int, 
    color: Tuple[int,int,int]=(0,0,255),
    thickness: int=1
):
    """
    วาดสี่เหลี่ยมจตุรัสขนาด side_len x side_len px
    ที่มุมซ้ายล่าง (px_slam, py_slam) ในพิกัด SLAM-toolbox
    ลงบนภาพ OpenCV img.
    """
    h, w = img.shape[:2]
    # มุมล่างซ้าย → OpenCV
    bl_x, bl_y = slam_to_opencv(px_slam, py_slam, h)
    # มุมบนขวา SLAM = (px_slam+side_len, py_slam+side_len) → OpenCV
    tr_x, tr_y = slam_to_opencv(px_slam + side_len, py_slam + side_len, h)
    
    # ในระบบ OpenCV: pt1 = top-left, pt2 = bottom-right
    top_left     = (bl_x, tr_y)
    bottom_right = (tr_x, bl_y)
    
    cv2.rectangle(img, top_left, bottom_right, color, thickness)

# ── ตัวอย่างการใช้งาน ──
if __name__ == "__main__":
    # === CONFIGURATION ===
    MAPS_DIR = "/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/maps"
    FILE_STEM = "ex1_obj_0v01_attemp2"
    YAML_PATH = os.path.join(MAPS_DIR, f"{FILE_STEM}.yaml")
    PGM_PATH  = os.path.join(MAPS_DIR, f"{FILE_STEM}.pgm")
    
    img = cv2.imread(PGM_PATH, cv2.IMREAD_COLOR)  # BGR
    h, w = img.shape[:2]

    resolution = 0.05  # meters/pixel
    px, py = -0.5, 2.0
    origin_x, origin_y = -10.1, -10.0
    pixelate_origin = [origin_x/resolution, origin_y/resolution]
    pixelate_p = [px/resolution, py/resolution]
    # สมมติ SLAM ให้จุด bottom-left = (50, 30), side=100 px
    slam_x, slam_y, side = (-pixelate_origin[0]+pixelate_p[0]), (-pixelate_origin[1]+pixelate_p[1]), 20
    print(slam_x, slam_y)
    
    expansion_lv = 0
    expansion_factor = 2 * expansion_lv
    # สร้าง mask
    
    draw_square(img, slam_x - expansion_lv, slam_y - expansion_lv, side + expansion_factor, color=(0,255,0), thickness=1)
    
    zoom_factor = 1  # อยากให้ขยายกี่เท่า
    # เลือกชื่อหน้าต่างตาม zoom
    window_name = "Zoomed Map" if zoom_factor != 1 else "Square on Map"

    if zoom_factor != 1:
        zoomed = cv2.resize(img, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(window_name, zoomed)
    else:
        cv2.imshow(window_name, img)

    # ลูปรอปิด
    while True:
        key = cv2.waitKey(100)
        if key == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()

