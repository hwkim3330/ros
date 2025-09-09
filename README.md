<img width="1679" height="773" alt="image" src="https://github.com/user-attachments/assets/56c8075d-e9c8-4d18-8dc5-37a54f4c5ae6" />
<img width="1612" height="909" alt="image" src="https://github.com/user-attachments/assets/8e2b9e6c-56a0-4507-87f9-adfaad9dcd1e" />
<img width="1764" height="1038" alt="image" src="https://github.com/user-attachments/assets/629b83fc-c928-467d-9a79-8d7882f3749d" />

# 🚀 Jetson Orin AGX ROS 2 + Gazebo 카메라 시뮬레이션 가이드

이 문서는 **ROS 2 Humble** 환경에서 **Gazebo Fortress + ros\_gz**를 활용하여 카메라 센서를 시뮬레이션하고, RViz2에서 실시간 영상을 확인하는 방법을 정리한 가이드입니다.

---

## 1. 사전 준비

### OS & ROS 버전 확인

```bash
lsb_release -a
# Ubuntu 22.04.x (Jammy) 확인

ros2 --version
# ros2 humble
```

### 워크스페이스/환경 변수

```bash
# ROS 2 기본 환경 설정
source /opt/ros/humble/setup.bash
```

---

## 2. 기존 Gazebo Classic 리포 제거

> Gazebo Classic(gazebo11)은 ARM64(Jetson)에서 공식 지원이 없어 설치가 실패합니다. 따라서 최신 **Gazebo Fortress** 조합으로 진행합니다.

```bash
sudo rm -f /etc/apt/sources.list.d/gazebo*.list
sudo rm -f /etc/apt/sources.list.d/osrf*.list
sudo apt update
```

---

## 3. 필요한 패키지 설치

### ros\_gz (ROS ↔ Gazebo 브리지) + 예제

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-sim-demos \
                    ros-humble-rviz2 ros-humble-image-view
```

* `ros-humble-ros-gz` : ROS 2 ↔ Gazebo(Fortress) 브리지
* `ros-humble-ros-gz-sim-demos` : 데모(카메라, 센서 등)
* `rviz2`, `image_view` : 시각화 도구

---

## 4. Gazebo 시뮬 실행

### 4.1 기본 월드 실행

```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

* 단순한 shapes 예제 월드가 실행됩니다.

### 4.2 카메라 데모 실행

새 터미널 열고:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim_demos camera.launch.py
```

* 가상 카메라 센서가 추가되어 `/camera/image` 토픽이 퍼블리시됩니다.

---

## 5. 토픽 확인

```bash
ros2 topic list | grep camera
# 예: /camera/camera_info
#     /camera/image
```

---

## 6. 화면 시각화

### RViz2

```bash
rviz2
```

* RViz2 실행 후 **Add → Image → Topic 선택**
* `/camera/image` 지정 → 카메라 영상 표시됨

### rqt\_image\_view

```bash
rqt_image_view
```

* 드롭다운에서 `/camera/image` 선택 → 영상 표시됨

---

## 7. 응용

* **실제 USB 카메라와 혼합**: `v4l2_camera` 패키지로 `/camera/image_raw` 퍼블리시 가능
* **다중 카메라**: SDF/launch 수정 후 ros\_gz 브리지 자동 반영
* **딥러닝 노드 연결**: `/camera/image`를 YOLO/Isaac ROS 등에 바로 연결 가능

---

## 8. 문제 해결 FAQ

* `gazebo_ros` 패키지를 못 찾음 → Gazebo Classic 대신 ros\_gz 사용 필요 (본 가이드 적용)
* `/camera/image`가 RViz2에 안 보임 → `ros2 topic list`로 실제 토픽명 확인 후 설정
* SSH 헤드리스에서 RViz2 실행 불가 → `rqt_image_view` 사용하거나 `image_saver`로 이미지 저장

  ```bash
  ros2 run image_view image_saver --ros-args -r image:=/camera/image
  ```

---

# ✅ 정리

* Jetson Orin + ROS 2 Humble에서는 Gazebo Classic 대신 **Gazebo Fortress + ros\_gz** 사용
* `ros-humble-ros-gz` + `ros-humble-ros-gz-sim-demos` 설치
* `gz_sim.launch.py`와 `camera.launch.py` 조합으로 가상 카메라 토픽 생성
* RViz2 / rqt\_image\_view에서 실시간 영상 확인 가능

# 🚗 Jetson Orin + ROS 2 + Gazebo 카메라 → 자율주행 맛보기 가이드

## 1. 준비 개요

* **ROS 2 Humble**
* **ros\_gz (Gazebo Fortress 브리지)**
* **카메라 센서 토픽** (`/camera/image`)
* **추가 센서**: 자율주행에는 라이다·Odometry가 필요하지만, 지금은 “카메라 기반”으로 갈 수 있는 **Lane Following / Object Detection 데모**부터.

---

## 2. 기본 토픽 확인

```bash
ros2 topic list | grep camera
# /camera/image
# /camera/camera_info
```

이 토픽이 정상 퍼블리시돼야 합니다.

---

## 3. OpenCV 기반 간단한 자율주행 데모 (차선 인식 → 스티어링)

1. 패키지 설치

```bash
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport \
                    python3-opencv
```

2. 예제 노드 작성 (`lane_follower.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image', self.callback, 10)
        self.get_logger().info("Lane follower node started")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        cv2.imshow("camera", frame)
        cv2.imshow("edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

3. 실행

```bash
chmod +x lane_follower.py
source /opt/ros/humble/setup.bash
ros2 run your_pkg lane_follower.py
```

* `/camera/image` → OpenCV 처리 → 차선 검출(edge) 시각화

---

## 4. 자율주행 시뮬 확장하기

### (1) 라이다 센서 추가

* Gazebo SDF에 `gpu_lidar` 센서 추가 가능
* ros\_gz 브리지에서 `/scan` 토픽 퍼블리시

### (2) Navigation2 스택 연결

* `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
* 맵 + 라이다 센서 필요
* 카메라 단독 주행은 어려우므로 보통 LiDAR+Odometry 같이 씁니다.

### (3) 자율주행 카트 예제

* **TurtleBot3 + Gazebo + Nav2** 예제 실행하면 “경로 계획 + 이동” 데모가 돌아갑니다:

```bash
sudo apt install ros-humble-turtlebot3* 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup navigation_launch.py
```

---

## 5. 카메라 기반 VLM/YOLO 연계

Jetson Orin에서 **YOLOv8/YOLO-3D**를 바로 `/camera/image` 토픽과 연결해

* 차선/차량 감지
* 신호등 색 인식
  같은 **인지 기반 자율주행** 모듈을 시도할 수 있습니다.

---

# ✅ 정리

1. 카메라 토픽(`/camera/image`) 확보 완료
2. OpenCV + ROS 2 노드로 간단한 **lane following** 시작
3. 이후 **라이다 + Nav2** 붙이면 풀 자율주행 가능
4. Jetson GPU 활용해 YOLO/Object Detection 연계 가능


