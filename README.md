# Hospital Lab Robot (MoMa) Project

병원 환경에서 검체를 운송하는 모바일 매니퓰레이터(Mobile Manipulator) 시뮬레이션 프로젝트입니다.

Nova Carter 베이스에 UR10 로봇팔을 결합하여, 자율 주행(Nav2)과 비전 인식(ArUco), 매니퓰레이션(RMPFlow)을 통합 제어합니다.

---

## 📂 디렉토리 구조 (Directory Structure)

이 프로젝트는 Monorepo 방식을 따르며, 시뮬레이션 에셋, 익스텐션 코드, ROS 2 워크스페이스를 통합 관리합니다.

```
hospital_robot_project/
├── assets/                 # [LFS] 3D 모델(USD) 및 텍스처 (Nova Carter, UR10, 병원 환경 등)
├── isaac_exts/             # NVIDIA Isaac Sim 커스텀 익스텐션 소스
│   └── rokey_lab_robot/    # (패키지명) 로봇 제어 및 시뮬레이션 환경 설정 코드
└── ros2_ws/                # ROS 2 워크스페이스
    └── src/
        └── my_pkg/         # Nav2, Vision, Master Node 패키지
```

---

## 🛠️ 사전 요구사항 (Prerequisites)

- **OS**: Ubuntu 22.04 LTS
- **Simulator**: NVIDIA Isaac Sim 4.2 또는 5.0 이상
- **Middleware**: ROS 2 Humble Hawksbill
- **Version Control**: Git & Git LFS (필수)

---

## 🚀 설치 및 설정 가이드 (Installation)

이 프로젝트를 처음 실행하는 개발자를 위한 단계별 가이드입니다.

### 1. 리포지토리 클론 및 LFS 데이터 다운로드

> ⚠️ **주의**: 3D 에셋(.usd) 파일들은 용량이 크기 때문에 Git LFS로 관리됩니다. 반드시 `lfs pull`을 실행해야 모델이 깨지지 않습니다.

```bash
# 프로젝트 클론
git clone <YOUR_REPOSITORY_URL> hospital_robot_project
cd hospital_robot_project

# LFS 설치 및 에셋 다운로드 (필수)
sudo apt install git-lfs
git lfs install
git lfs pull
```

### 2. ROS 2 패키지 빌드

ROS 2 노드 실행을 위한 의존성 설치 및 빌드를 진행합니다.

```bash
cd ~/hospital_robot_project/ros2_ws

# 의존성 설치
rosdep install -i --from-path src --rosdistro humble -y

# 빌드
colcon build --symlink-install

# 환경 설정 (터미널마다 실행 필요)
source install/setup.bash
```

### 3. Isaac Sim 익스텐션 경로 등록 (⭐ 중요)

Isaac Sim이 이 프로젝트의 파이썬 코드를 인식하도록 Search Path를 등록해야 합니다.

1. Isaac Sim 실행
2. 상단 메뉴: **Window → Extensions** 클릭
3. Extensions 창 우측 상단의 **톱니바퀴 아이콘** (Settings) 클릭
4. **[Extension Search Paths]** 목록의 `+` 버튼 클릭 후 아래 경로 추가:

```
/home/<사용자명>/hospital_robot_project/isaac_exts
```

> ⚠️ **주의**: `rokey_lab_robot` 폴더가 아니라 그 상위 폴더인 `isaac_exts`를 지정해야 합니다.

5. 검색창에 `Hospital` 입력 → **[Hospital Lab Robot]** 익스텐션 확인
6. Toggle 스위치 **ON** (초록색) & **Autoload** 체크

---

## ▶️ 실행 방법 (Usage)

### 1. 시뮬레이션 실행

1. Isaac Sim 상단 메뉴: **Window → Isaac Examples** (또는 **Isaac Utils → Workflows**)
2. **[My Projects]** 탭 클릭
3. **[Hospital Lab Robot]** 버튼 클릭 → **Load**
4. 로봇과 병원 환경이 로드되면, 좌측 툴바의 **PLAY (▶)** 버튼을 눌러 시뮬레이션 시작
5. 콘솔에 `📡 [ROS 2] Waiting for commands...` 메시지가 뜨면 성공

### 2. ROS 2 노드 실행 (터미널)

시뮬레이션이 실행 중인 상태에서 새로운 터미널을 열고 명령을 내립니다.

#### Nav2 네비게이션 명령 노드:

```bash
cd ~/hospital_robot_project/ros2_ws
source install/setup.bash
ros2 run my_pkg nav_commander
```

#### ArUco 비전 인식 노드:

```bash
cd ~/hospital_robot_project/ros2_ws
source install/setup.bash
ros2 run my_pkg aruco_detector
```

---

## ⚠️ 문제 해결 (Troubleshooting)

### Q1. Isaac Sim에서 로봇이 보라색으로 나오거나 모델이 안 보입니다.

**원인**: Git LFS가 제대로 설치되지 않아 USD 파일의 포인터만 다운로드된 경우입니다.

**해결**: `git lfs pull` 명령어를 다시 실행하고, `assets` 폴더 내의 파일 용량이 1KB 이상인지 확인하세요.

### Q2. Extensions 메뉴에 'Hospital Lab Robot'이 안 뜹니다.

**원인**: Search Path 설정이 잘못되었습니다.

**해결**: 경로가 `.../isaac_exts`로 끝나는지 확인하세요. `.../rokey_lab_robot`까지 들어가면 안 됩니다.

### Q3. 코드 실행 시 FileNotFoundError가 발생합니다.

**원인**: 코드 내에 절대 경로(`/home/jy/...`)가 하드코딩 되어 있습니다.

**해결**: `isaac_exts/rokey_lab_robot/rokey_lab_robot/lab_robot_main.py` 파일을 열고, `user_usd_path` 변수의 경로를 본인의 사용자명(`<username>`)에 맞게 수정해주세요.

---

## 📝 License

This project is licensed under the Apache 2.0 License.

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!

---

## 👤 Author

**박주영 (Park Juyoung)**  
Korea University - Robotics Engineering Student

---

## 🙏 Acknowledgments

- NVIDIA Isaac Sim
- ROS 2 Community
- Nav2 Navigation Stack
