# 🏥 Hospital Robot Control System

병원 서비스 로봇을 위한 통합 제어 시스템

## 📋 시스템 구성

```
┌─────────────────────────────────────────────────────┐
│                   Web Browser                       │
│              (hospital_robot_ui.html)               │
└──────────────────────┬──────────────────────────────┘
                       │ WebSocket + HTTP
┌──────────────────────▼──────────────────────────────┐
│              ROS2-UI Bridge Server                  │
│           (ros2_ui_bridge.py + FastAPI)             │
│              + Hospital Database                    │
└──────────────────────┬──────────────────────────────┘
                       │ ROS2 Action
┌──────────────────────▼──────────────────────────────┐
│            Main Controller Node                     │
│             (main_controller.py)                    │
└──────────────────────┬──────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
    ┌───────┐    ┌─────────┐    ┌─────────┐
    │ Nav2  │    │ Docking │    │ Vision  │
    └───────┘    └─────────┘    └─────────┘
```

## 🗂️ 파일 구조

```
hospital_robot_system/
├── hospital_robot_db.py        # 데이터베이스 관리 (SQLite)
├── ros2_ui_bridge.py           # ROS2-UI 브릿지 서버 (FastAPI)
├── hospital_robot_ui.html      # 웹 제어 인터페이스 (React)
├── main_controller.py          # 메인 컨트롤러 노드 (기존)
└── README.md                   # 이 문서
```

## 📊 데이터베이스 스키마

### 1. **rooms** - 방 정보
- 병원 내 모든 방의 좌표, 방향, 작업면 정의
- Zone A (환자 구역), B (약제 구역), C (지원 구역)

### 2. **items** - 물품 정보
- 마커 ID, 오프셋, 카테고리 정보
- Blood Sample, Medicine, Narcotics 등

### 3. **delivery_tasks** - 배송 작업 이력
- 작업 모드, 상태, 소요 시간, 배터리 소모량 추적
- PENDING → IN_PROGRESS → COMPLETED/FAILED

### 4. **robot_status_log** - 로봇 상태 로그
- 실시간 위치, 배터리, 동작 상태 기록
- 작업별 상세 로그 추적

### 5. **system_config** - 시스템 설정
- 홈 위치, 배터리 소모율 등 설정 값

### 6. **camera_snapshots** - 카메라 스냅샷 (선택)
- 작업 중 캡처된 이미지 메타데이터

## 🚀 설치 및 실행

### 1. 필수 패키지 설치

```bash
# Python 패키지
pip install fastapi uvicorn websockets opencv-python numpy scipy --break-system-packages

# 또는 requirements.txt로 설치
cat > requirements.txt << EOF
fastapi==0.104.1
uvicorn[standard]==0.24.0
websockets==12.0
opencv-python==4.8.1.78
numpy==1.24.3
scipy==1.11.4
python-multipart==0.0.6
EOF

pip install -r requirements.txt --break-system-packages
```

### 2. 데이터베이스 초기화

```bash
# 데이터베이스 생성 및 초기 데이터 삽입
python3 hospital_robot_db.py

# 성공 시 출력:
# ✅ Database initialized successfully!
# ✅ Initial data populated successfully!
```

### 3. ROS2 노드 실행 (메인 컨트롤러)

```bash
# 터미널 1: 메인 컨트롤러
python3 main_controller.py

# 출력:
# 🏥 Hospital Main Node Ready (Waiting for UI Command...)
```

### 4. 브릿지 서버 실행

```bash
# 터미널 2: ROS2-UI 브릿지
python3 ros2_ui_bridge.py

# 출력:
# 🚀 Hospital Robot Bridge Server Started!
#    - HTTP API: http://localhost:8000
#    - WebSocket: ws://localhost:8000/ws
#    - Docs: http://localhost:8000/docs
```

### 5. UI 접속

```bash
# 웹 브라우저에서 HTML 파일 열기
firefox hospital_robot_ui.html
# 또는
google-chrome hospital_robot_ui.html
```

## 🎮 UI 사용법

### 메인 제어 패널 (좌측)
1. **Item Type** 선택: Blood Sample, Medicine, Narcotics
2. **Pickup Location** 선택: 출발지 (예: Nurse Station A)
3. **Dropoff Location** 선택: 목적지 (예: Clinical Lab)
4. **Run Full Delivery**: 전체 시나리오 실행 (NAV → DOCK → PICK → NAV → DOCK → PLACE → HOME)

### 개별 기능 버튼
- **NAV Pickup**: 픽업 위치로만 이동
- **DOCK Pickup**: 도킹만 실행
- **PICK**: 픽업만 실행
- **NAV Dropoff**: 하역지로만 이동
- **DOCK Dropoff**: 하역지 도킹만 실행
- **PLACE**: 내려놓기만 실행
- **NAV Home**: 홈으로 복귀
- **ARM Home**: 팔만 홈 위치로

### 카메라 피드 (중앙)
- **FRONT Camera**: 일반 주행 시 표시
- **LEFT Camera**: 왼쪽 작업 시 ArUco 마커 디버그
- **RIGHT Camera**: 오른쪽 작업 시 ArUco 마커 디버그

### 실시간 로그 (우측 상단)
- 모든 작업 단계 상세 로그 표시
- 색상 코드:
  - 🔵 파란색: 정보 (INFO)
  - 🟢 초록색: 성공 (SUCCESS)
  - 🟡 노란색: 경고 (WARNING)
  - 🔴 빨간색: 오류 (ERROR)

### 통계 대시보드 (좌측 하단)
- **Total Tasks**: 전체 작업 수
- **Today's Tasks**: 오늘 수행한 작업
- **Avg Battery Use**: 평균 배터리 소모량
- **Avg Duration**: 평균 작업 소요 시간

### 작업 이력 (우측 하단)
- 최근 10개 작업 표시
- 상태별 색상:
  - 🟢 초록: COMPLETED
  - 🔵 파란: IN_PROGRESS
  - 🔴 빨강: FAILED
  - 🟠 주황: PENDING

## 📡 API 엔드포인트

브릿지 서버는 다음 REST API를 제공합니다:

### 조회 (GET)
```bash
# 방 목록
curl http://localhost:8000/api/rooms

# 물품 목록
curl http://localhost:8000/api/items

# 작업 이력 (최근 50개)
curl http://localhost:8000/api/tasks?limit=50

# 특정 작업 조회
curl http://localhost:8000/api/tasks/1

# 통계
curl http://localhost:8000/api/statistics

# 로봇 상태
curl http://localhost:8000/api/status
```

### 제어 (POST)
```bash
# 새 작업 생성
curl -X POST http://localhost:8000/api/tasks/create \
  -H "Content-Type: application/json" \
  -d '{
    "task_mode": "ALL",
    "item_type": "Blood Sample",
    "pickup_loc": "Nurse Station A (Base)",
    "dropoff_loc": "Clinical Lab (Zone C)"
  }'

# 배터리 리셋 (테스트용)
curl -X POST http://localhost:8000/api/battery/reset
```

## 🔧 설정 커스터마이징

### 1. 방(Room) 추가/수정
```python
# hospital_robot_db.py의 populate_initial_data() 함수 수정
db.insert_room(
    room_name="New Room",
    zone="A",
    coord_x=10.0,
    coord_y=5.0,
    coord_z=0.0,
    direction="East",
    work_side="Left",
    description="새로운 방"
)
```

### 2. 물품(Item) 추가/수정
```python
# hospital_robot_db.py의 populate_initial_data() 함수 수정
db.insert_item(
    item_name="New Item",
    marker_id=10,
    offset_x=0.0,
    offset_y=0.05,
    offset_z=-0.03,
    category="Medical",
    description="새 물품"
)
```

### 3. 배터리 소모율 조정
```python
# ros2_ui_bridge.py의 odom_callback() 함수에서
battery_drain = distance * 0.5  # 0.5를 원하는 값으로 변경
```

### 4. 카메라 업데이트 주기 조정
```python
# ros2_ui_bridge.py의 websocket_endpoint() 함수에서
await asyncio.sleep(0.5)  # 0.5초를 원하는 값으로 변경 (초 단위)
```

## 🐛 문제 해결

### 1. "Action server not available" 오류
```bash
# main_controller.py가 실행 중인지 확인
ps aux | grep main_controller

# ROS2 노드 목록 확인
ros2 node list

# Action 서버 확인
ros2 action list
```

### 2. 데이터베이스 연결 오류
```bash
# 데이터베이스 파일 권한 확인
ls -l hospital_robot.db

# 데이터베이스 재생성
rm hospital_robot.db
python3 hospital_robot_db.py
```

### 3. WebSocket 연결 실패
```bash
# 브릿지 서버 포트 확인
netstat -tulpn | grep 8000

# 방화벽 설정 확인
sudo ufw allow 8000
```

### 4. 카메라 피드가 표시되지 않음
```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# 카메라 토픽 메시지 확인
ros2 topic echo /front_camera/rgb --once
```

## 📈 데이터 분석 (향후 확장)

데이터베이스를 활용한 추가 기능:

### 1. 작업 효율 분석
```sql
-- 시간대별 작업 분포
SELECT strftime('%H', started_at) as hour, COUNT(*) as task_count
FROM delivery_tasks
WHERE DATE(started_at) = DATE('now')
GROUP BY hour;

-- 경로별 평균 시간
SELECT pickup_location, dropoff_location, 
       AVG(duration_seconds) as avg_duration
FROM delivery_tasks
WHERE status = 'COMPLETED'
GROUP BY pickup_location, dropoff_location;
```

### 2. 배터리 최적화
```sql
-- 배터리 소모량 분석
SELECT item_name, 
       AVG(battery_start - battery_end) as avg_consumption,
       COUNT(*) as task_count
FROM delivery_tasks
WHERE battery_end IS NOT NULL
GROUP BY item_name;
```

### 3. 오류 패턴 분석
```sql
-- 실패 원인별 통계
SELECT error_message, COUNT(*) as error_count
FROM delivery_tasks
WHERE status = 'FAILED'
GROUP BY error_message
ORDER BY error_count DESC;
```

## 🎯 시나리오 예시

### 시나리오 1: 혈액 검체 운반
```
1. UI에서 선택:
   - Item: Blood Sample
   - Pickup: Nurse Station A (Base)
   - Dropoff: Clinical Lab (Zone C)

2. "Run Full Delivery" 클릭

3. 자동 실행 단계:
   ✅ NAV_PICKUP: 간호사 스테이션으로 이동
   ✅ DOCK_PICKUP: 정밀 도킹
   ✅ PICK: 왼쪽 카메라로 마커 인식 → 혈액 샘플 픽업
   ✅ NAV_DROPOFF: 검사실로 이동
   ✅ DOCK_DROPOFF: 정밀 도킹
   ✅ PLACE: 오른쪽 카메라로 검증 → 샘플 내려놓기
   ✅ NAV_HOME: 시작 위치 복귀
```

### 시나리오 2: 약품 배송 (단계별)
```
1. NAV Pickup → 약제실로 이동만
2. DOCK Pickup → 도킹만
3. PICK → 약품 픽업만
4. (점심시간 대기...)
5. NAV Dropoff → 병실로 이동
6. DOCK Dropoff → 도킹
7. PLACE → 약품 배송
8. NAV Home → 복귀
```

## 📞 기술 지원

문제가 발생하면 다음을 확인하세요:

1. **로그 확인**: UI의 System Logs 패널
2. **터미널 출력**: main_controller.py 및 ros2_ui_bridge.py
3. **데이터베이스**: `sqlite3 hospital_robot.db`로 직접 조회
4. **API 문서**: http://localhost:8000/docs (Swagger UI)

## 🎉 완성!

이제 병원 로봇 시스템이 완전히 준비되었습니다!
- ✅ 데이터베이스 (SQLite)
- ✅ ROS2 브릿지 서버 (FastAPI + WebSocket)
- ✅ 웹 UI (React)
- ✅ 실시간 카메라 피드
- ✅ 작업 이력 추적
- ✅ 통계 대시보드

Happy Robot Controlling! 🤖
