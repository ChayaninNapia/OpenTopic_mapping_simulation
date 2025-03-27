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

## หลักการของ cartographer_ros
- เป็นกราฟ SLAM อีกแบบหนึ่งที่พัฒนาโดย Google โดยใช้การแบ่ง submap