# OpenTopic_mapping_simulation

## create your workspace
```
cd
mkdir mappig_ws && cd mapping_ws
```

### clone this repository
```
git clone ...
```
```
cd ~/mapping_ws/

# Clone mir_robot into the ROS2 workspace
git clone -b humble-devel https://github.com/relffok/mir_robot src/mir_robot

# Fetch linked repositories using vcs
vcs import < src/mir_robot/ros2.repos src --recursive

# Install dependencies using rosdep (including ROS)
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build all packages in the workspace
cd ~/mapping_ws
colcon build
```

## Algortihm package ที่จะเปรียบเทียบ
หมายเหตุ: ตอนนี้ดูเหมือนว่าแต่ละ package จะไม่ได้ใช้แค่อัลกอริทึมหลักตัวเดียวแบบตรงไปตรงมา แต่มีการปรับแต่งและเพิ่มการทำ optimization เข้าไปด้วย ทำให้ไม่สามารถมองแค่ตัวอัลกอริทึมเดี่ยว ๆ ได้ ก็เลยจะใช้คำว่า "เปรียบเทียบ package" แทน เพื่อให้ครอบคลุมถึงฟีเจอร์และการปรับปรุงที่แต่ละตัวมี

- filter-based
    1. slam_gmapping

- graph-based
    1. slam-toolbox
    2. cartographer_ros

## หลักการของ slam_gmapping
- ใช้วิธี filter-based โดยใช้ Rao-Blackwellized particle filter<br> ไอเดียคือ " ถ้าเราสามารถคำนวณบางตัวแปรได้แบบวิเคราะห์ ก็สุ่มแค่บางส่วนพอ "
- SLAM algorithm ที่สร้างบนพื้นฐานของ RBPF ชื่อว่า FastSLAM <br>
ใช้ particle แต่ละตัวแทน “trajectory” และมี “map” แยกของตัวเอง
- ปรับปรุงจาก FastSLAM -> " Gmapping "
    - ใช้ scan matching + observation เพื่อสร้าง proposal distribution ที่แม่นขึ้น
    - ใช้ adaptive resampling ลดปัญหา particle depletion
### Rao-Blackwellized particle filter คืออะไร?


## หลักการของ slam-toolbox
- เป็นกราฟ SLAM (graph-based SLAM) ที่พัฒนามาจาก KartoSLAM

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_slam_toolbox/config/mapper_params_online_async.yaml
```
### รูปแบบของไฟล์ map ของ slam-toolbox
| ประเภทไฟล์       | ใช้ทำอะไร |
|------------------|------------|
| **Occupancy Grid** | `.pgm` (ภาพ) + `.yaml` (metadata) — ใช้สำหรับ Nav2 และ map_server |
| **Pose Graph**     | `.posegraph` — เก็บข้อมูลกราฟ SLAM เพื่อโหลดมาต่อ mapping หรือ localization ต่อ |
| **Full State**     | `.data` — บันทึก state ทั้งหมด (nodes, constraints, metadata) สำหรับโหลดระบบคืนทั้งหมด |

### ตัวแปรต้นที่ต้องการจะเปรียบทเียบระหว่าง online async vs lifelong

## 🔍 Metrics for Comparing Online Async vs Lifelong Modes in SLAM Toolbox

### 1. Pose Estimation Error
- **1.1 Translational RMSE**: ความคลาดเคลื่อนในตำแหน่ง (เช่น, ค่า RMSE ระยะทาง)
- **1.2 Angular RMSE**: ความคลาดเคลื่อนในมุม (เช่น, ค่า RMSE มุม)
> 🛠 ใช้ plot แบบ time series หรือ histogram เพื่อดูว่าความแม่นยำของตำแหน่งเปลี่ยนไปอย่างไรในแต่ละโหมด

### 2. Pose Graph Characteristics
- **2.1 จำนวน Node**: จำนวนของโหนดใน pose graph ที่ถูกเพิ่มเข้ามา (lifelong มีการ prune node ด้วย)
- **2.2 จำนวน Constraint (Edge)**: จำนวนของ connection ระหว่างโหนดที่แสดงถึงการจับคู่และ loop closure
- **2.3 ค่า Cost ใน Graph Optimization**: ตัวชี้วัด error ทั้งหมดใน pose graph ที่ถูก optimize
> 📈 Visualize ผ่าน node/edge count และ graph cost timeline

### 3. Occupancy Grid Map Quality
- **3.1 Map Consistency / Accuracy**: เปรียบเทียบกับ ground truth หรือแผนที่จากโหมดอื่น
- **3.2 Difference Image**: แสดงความเปลี่ยนแปลงในแผนที่ระหว่างโหมด
> 🌡 ใช้ heatmap หรือ overlay image เพื่อแสดงความต่าง

### 4. Loop Closure Frequency & Impact
- **4.1 จำนวน Loop Closures**: นับจำนวน loop closure ที่เกิดขึ้น
- **4.2 ผลกระทบต่อ Optimization**: เช่น ความเปลี่ยนแปลงของ graph cost หลังเกิด loop closure
> 🧭 ใช้ timeline plot แสดงช่วงเวลาและผลกระทบของแต่ละ loop closure

### 5. Computational Performance
- **5.1 Processing Time per Scan**: เวลาที่ใช้ประมวลผลข้อมูลในแต่ละ scan
- **5.2 CPU / Memory Usage**: การใช้ทรัพยากรในแต่ละโหมด
> ⚙️ Plot แบบ bar chart หรือ time series

### 6. Update Frequency / Latency
- **6.1 Map Update Interval**: ความถี่ในการอัปเดต occupancy grid
- **6.2 Latency ของการ Publish / Update**: ความล่าช้าในการแสดงผลแผนที่
> ⏱ ใช้ average + distribution plot วิเคราะห์ real-time performance

### 7. Node Pruning Metrics (เฉพาะ Lifelong Mode)
- **7.1 Rate of Node Removal**: อัตราการตัด node ออกจาก graph
- **7.2 อายุเฉลี่ยของ Node ที่คงเหลือ**: เพื่อดูพฤติกรรม decay node เก่า
> 🌿 Time series แสดงการเปลี่ยนแปลงจำนวน node และอายุ node


## Pose Estimation Error setup



## หลักการของ cartographer_ros
- เป็นกราฟ SLAM อีกแบบหนึ่งที่พัฒนาโดย Google โดยใช้การแบ่ง submap
- เก็บ map ในรูปแบบ Probability Grid 

