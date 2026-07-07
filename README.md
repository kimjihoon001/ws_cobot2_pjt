# Collaborative Robot Smart Assembly & Quality Inspection System
> Speech Control, YOLOv8, and Depth Vision based safe HRI system using Doosan Robotics m0609 & OnRobot RG2.

본 프로젝트는 두산로보틱스 m0609 협동로봇과 OnRobot RG2 그리퍼, Intel RealSense 카메라를 결합하여 음성 제어 및 실시간 YOLO 비전/깊이 기반의 스마트 조립 및 장애물 회피 구동을 수행하는 시스템입니다.

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

## 💻 1. 가상 환경 (Virtual Mode) 실행 방법
실제 로봇 연결 없이 PC 시뮬레이션 환경에서 연동 및 회피 경로 생성을 검증하는 방법입니다.

### 터미널 1: RealSense 카메라 구동
```bash
realsense
```

### 터미널 2: 가상 로봇 드라이버 및 RViz 기동
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=virtual
```
*(로봇 하단에 설치된 실제 사이즈의 회색 작업대 테이블이 함께 로드됩니다.)*

### 터미널 3: MoveIt 모션 플래너 기동
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 launch m0609_rg2_moveit movegroup_only.launch.py
```

### 터미널 4: YOLOv8 비전 노드 실행 (GPU/CUDA 가속)
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 run object_detection object_detection
```
*(기동 시 `YOLO initialized on device: cuda` 문구가 나타납니다.)*

### 터미널 5: 장애물 변환 및 지속 기억 노드 실행
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 run object_detection tool_obstacle_publisher --ros-args -p target_tool:=screwdriver
```
*(감지 상실 시에도 4.0초간 최종 손 위치를 유지하여 회피 기동을 보장합니다.)*

### 터미널 6: 왕복 모션 및 회피 루프 실행
```bash
source ~/ws_cobot2_pjt/install/setup.bash
python3 src/cobot2_ws/m0609_rg2_bringup/scripts/hand_avoidance.py
```

---

## 🦾 2. 실제 로봇 환경 (Real Mode) 실행 방법
실제 Doosan m0609 로봇(IP: `192.168.1.100`) 하드웨어에 직결하여 비전 회피 및 조작을 기동하는 방법입니다.

> [!IMPORTANT]
> **준비 사항**: PC와 두산 로봇 컨트롤러는 반드시 **유선 랜선(이더넷)**으로 연결되어 있어야 제어 지연이 생기지 않습니다.

### 터미널 1: RealSense 카메라 구동
```bash
realsense
```

### 터미널 2: 실제 로봇 드라이버 기동 (IP: 192.168.1.100)
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100
```

### 터미널 3: MoveIt 모션 플래너 기동
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 launch m0609_rg2_moveit movegroup_only.launch.py
```

### 터미널 4: YOLOv8 비전 노드 실행 (GPU 가속)
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 run object_detection object_detection
```

### 터미널 5: 장애물 변환 및 지속 기억 노드 실행
```bash
source ~/ws_cobot2_pjt/install/setup.bash
ros2 run object_detection tool_obstacle_publisher --ros-args -p target_tool:=screwdriver
```

### 터미널 6: 왕복 모션 및 회피 루프 실행
```bash
source ~/ws_cobot2_pjt/install/setup.bash
python3 src/cobot2_ws/m0609_rg2_bringup/scripts/hand_avoidance.py
```

---

## 🧱 3. 젠가 불량 검사 및 실시간 손 회피 시스템 실행 방법 (Real Mode)
실제 로봇 환경에서 YOLOv8 및 RealSense 카메라를 결합한 젠가 불량 검사 시퀀스를 기동하고, 동작 중 ROI 기반의 손 회피 기능까지 연동하여 작동시키는 가이드입니다.

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
