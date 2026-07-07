# Collaborative Robot Smart Assembly & Quality Inspection System
> Speech Control, YOLOv8, and Depth Vision based safe HRI system using Doosan Robotics m0609 & OnRobot RG2.

본 프로젝트는 두산로보틱스 m0609 협동로봇과 OnRobot RG2 그리퍼, Intel RealSense 카메라를 결합하여 음성 제어 및 실시간 YOLO 비전/깊이 기반의 스마트 조립 및 장애물 회피 구동을 수행하는 시스템입니다.

---

## 📂 주요 패키지 구성 요약
본 시스템은 다음과 같은 핵심 ROS 2 패키지들로 구성되어 있습니다:
* **[robot_control](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/robot_control)**: 젠가 스캔 제어, MoveIt 조인트 명령 및 ROI 손 감지 콜백을 처리하는 메인 제어부 노드 (`jenga_inspector.py`)
* **[object_detection](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/object_detection)**: RealSense 카메라 연동 및 YOLOv8 온디바이스 젠가 검출 서비스 노드 (`jenga_detection.py`)
* **[object_hand](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/object_hand)**: YOLO 기반 손 3D 위치 추적 및 MoveIt용 5cm 구체 장애물 발행 노드
* **[m0609_rg2_bringup](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup)**: 실제 두산 로봇 드라이버 및 카메라 통합 기동 런치파일 및 URDF 로드
* **[m0609_rg2_moveit](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_moveit)**: MoveIt 2 모션 플래닝 설정 관련 설정 및 런치파일
* **[od_msg](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/od_msg)**: 노드 간 좌표 통신에 쓰이는 사용자 정의 ROS 2 서비스 인터페이스 정의
* **[onrobot-ros2](file:///home/rokey/ws_cobot2_pjt/src/cobot2_ws/onrobot-ros2)**: OnRobot RG2 그리퍼 제어 및 Modbus TCP 통신 드라이버

---

## 🛠️ 하드웨어 구성
- **로봇 암**: Doosan Robotics m0609 (네임스페이스: `/dsr01`, 제어 IP: `192.168.1.100`)
- **그리퍼**: OnRobot RG2 (기본 연결 IP: `192.168.1.1`)
- **카메라**: Intel RealSense D435i/D435Y (그리퍼에 마운트된 Eye-in-Hand 방식)

---

## 🏗️ 빌드 방법 (Build Instructions)

워크스페이스 변경사항 반영 및 컴파일을 수행합니다.

```bash
# 1. 워크스페이스 폴더로 이동
cd ~/ws_cobot2_pjt

# 2. 의존 패키지 빌드
colcon build --symlink-install

# 3. 환경 변수 적용
source install/setup.bash
```

---

## 🦾 1. YOLO 공구 인식 및 자동 파지 시스템 실행 방법

YOLO + RealSense 깊이 카메라로 해머를 감지하고, MoveIt 관절 공간 제어(Joint Space Control)로 자동 파지 후 들어 올리는 시스템입니다.

> [!IMPORTANT]
> **준비 사항**: PC와 두산 로봇 컨트롤러는 반드시 **유선 랜선(이더넷)**으로 연결되어 있어야 합니다. 로봇 펜던트의 동작 모드는 **자동(Auto)** 상태로 설정해 주세요.

### 터미널 1: RealSense 카메라 구동
```bash
realsense
```
*(`/camera/camera/color/image_raw` 및 `/camera/camera/aligned_depth_to_color/image_raw` 토픽이 발행되어야 합니다.)*

### 터미널 2: 실제 로봇 드라이버 기동 (IP: 192.168.1.100)
```bash
cd ~/ws_cobot2_pjt && source install/setup.bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100
```
*(가상 시뮬레이션만 하는 경우: `mode:=virtual`로 변경)*

### 터미널 3: MoveIt 모션 플래너 기동
```bash
cd ~/ws_cobot2_pjt && source install/setup.bash
ros2 launch m0609_rg2_moveit movegroup_only.launch.py
```

### 터미널 4: OnRobot RG2 그리퍼 드라이버 기동
```bash
cd ~/ws_cobot2_pjt && source install/setup.bash
ros2 launch onrobot_rg_ros2 bringup.launch.py ip:=192.168.1.1
```

### 터미널 5: YOLO 파지 스크립트 실행
```bash
cd ~/ws_cobot2_pjt && source install/setup.bash
python3 src/cobot2_ws/m0609_rg2_bringup/scripts/tool_pick_yolo_target.py
```

실행 흐름:
1. 카메라로 `tool-hammer` 클래스를 탐지 (신뢰도 0.7 이상)
2. 깊이 센서로 실제 3D 좌표(`base_link` 기준) 계산
3. MoveIt Planning Scene에 해머 STL 메쉬를 장애물로 스폰
4. 관절 공간 IK(Joint Space Control)로 pregrasp → grasp 이동
5. RG2 그리퍼 닫기 → 파지 완료 → 들어 올리기 (pregrasp 복귀)

---


## 🧱 3. 젠가 불량 검사 및 실시간 손 회피 시스템 실행 방법 (Real Mode)
실제 로봇 환경에서 YOLOv8 및 RealSense 카메라를 결합한 젠가 불량 검사 시퀀스(측정 각도 30°, 37°, 45°, 52°, 60° 다중 촬영 및 최적 2면 선정)를 기동하고, 동작 중 ROI 기반의 손 회피 기능까지 연동하여 작동시키는 가이드입니다.

### 터미널 1: RealSense 카메라 구동
```bash
realsense
```

### 터미널 2: 실제 로봇 드라이버 기동 (IP: 192.168.1.100)
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100
```

### 터미널 3: MoveIt 모션 플래너 기동
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 launch m0609_rg2_moveit movegroup_only.launch.py
```

### 터미널 4: YOLO 기반 젠가 타워 및 불량 검사 비전 인식 서비스 구동
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 run object_detection jenga_detection
```

### 터미널 5: YOLO 기반 손 검출 인식 서비스 노드 구동
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 run object_hand object_hand
```

### 터미널 6: 실시간 손 장애물 토픽 퍼블리셔 기동
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 run object_hand hand_obstacle_publisher
```

### 터미널 7: 젠가 불량 검증 및 손 회피 메인 제어 노드 구동
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 run robot_control jenga_inspector
```

### 터미널 8: 불량 검사 서비스 호출 (Trigger)
모든 노드가 정상 기동된 상태에서 아래 명령어를 실행하여 젠가 불량 검사 및 손 회피 테스트 시퀀스를 시작합니다.
```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 service call /run_jenga_inspection std_srvs/srv/Trigger
```

---

## 🔍 문제 해결 및 문제 조치 (Troubleshooting)

### Q1. 로봇이 움직일 때 덜컹거리는 진동이나 렉이 발생합니다.
- **해결책 1**: 터미널 4번의 YOLO 노드가 GPU(`cuda`)를 정상적으로 사용 중인지 로그를 확인하세요. CPU로 구동 시 100% 점유율 폭발로 인해 실시간 관절 스트리밍 주기가 밀려 덜컹거림이 발생합니다.
- **해결책 2**: PC가 Wi-Fi 대신 기가비트 이더넷 랜선으로 로봇에 단단히 연결되어 있는지 확인해 주세요.

### Q2. `Spawner` 관련 에러가 발생하고 RViz 구동이 먹통이 됩니다.
- **원인**: 이전 구동 노드가 백그라운드에 좀비로 살아있어 포트/DDS 충돌이 난 상태입니다.
- **해결책**: 터미널에 아래 명령어를 내려 충돌 요소를 모두 밀고 재시작합니다.
  ```bash
  killall -9 ros2_control_node rviz2 spawner || true
  ```
