# 🚀 빠른 시작 가이드 (Quick Start)

## ⚡ 30초 안에 시작하기

### 1단계: 권한 부여
```bash
chmod +x launch_system.sh
```

### 2단계: 시스템 실행
```bash
./launch_system.sh
```

### 3단계: 옵션 선택
```
Select launch option:
  1) Launch Bridge Server only
  2) Launch Everything (전체 시스템 실행 - 권장) ⭐
  3) Test Database
  4) Exit

Enter option [1-4]: 2
```

### 4단계: 브라우저에서 UI 열기
```bash
# 자동으로 열리지 않으면:
firefox hospital_robot_ui.html
# 또는
google-chrome hospital_robot_ui.html
```

---

## 🎮 첫 번째 작업 실행하기

### UI에서 다음을 선택:
1. **Item Type**: Blood Sample
2. **Pickup Location**: Nurse Station A (Base)
3. **Dropoff Location**: Clinical Lab (Zone C)
4. **[Run Full Delivery]** 버튼 클릭

### 예상 결과:
```
✅ Task #1 created
📍 NAVIGATING TO PICKUP
📍 DOCKING AT PICKUP
📍 PICKING
📍 NAVIGATING TO DROPOFF
📍 DOCKING AT DROPOFF
📍 PLACING
📍 RETURNING HOME
✅ Task Completed
```

---

## 📊 주요 기능

### 전체 시나리오 실행
- **Run Full Delivery**: 픽업 → 배송 → 복귀까지 한 번에

### 단계별 실행
- **NAV Pickup**: 픽업 장소로 이동만
- **DOCK Pickup**: 도킹만
- **PICK**: 물건 집기만
- **NAV Dropoff**: 배송지로 이동만
- **DOCK Dropoff**: 배송지 도킹만
- **PLACE**: 물건 내려놓기만
- **NAV Home**: 홈으로 복귀만
- **ARM Home**: 팔만 홈으로

### 실시간 모니터링
- 🔋 **배터리**: 실시간 배터리 잔량 (이동에 따라 감소)
- 📹 **카메라**: Front/Left/Right 3개 카메라 피드
- 📋 **로그**: 모든 작업 단계 상세 로그
- 📊 **통계**: 작업 수, 평균 시간, 배터리 소모량

---

## 🛑 시스템 종료

### tmux 사용 시:
```bash
tmux kill-session -t hospital_robot
```

### 일반 실행 시:
```bash
Ctrl + C
```

---

## 🐛 문제 해결

### "Action server not available"
```bash
# main_controller.py가 실행 중인지 확인
ros2 node list | grep hospital_main_node
```

### "Database not found"
```bash
# 데이터베이스 재생성
python3 hospital_robot_db.py
```

### "Connection Error"
```bash
# 브릿지 서버가 실행 중인지 확인
curl http://localhost:8000/api/status
```

---

## 📁 파일 위치

```
현재 디렉토리/
├── hospital_robot_db.py       ✅ 데이터베이스
├── ros2_ui_bridge.py          ✅ 브릿지 서버
├── hospital_robot_ui.html     ✅ 웹 UI
├── main_controller.py         ✅ 메인 노드 (기존)
├── launch_system.sh           ✅ 실행 스크립트
├── requirements.txt           ✅ 패키지 목록
├── README.md                  📖 상세 가이드
└── QUICKSTART.md             ⚡ 이 문서
```

---

## 🎯 추천 시나리오

### 시나리오 1: 검체 운송 (전체)
```
Item: Blood Sample
From: Nurse Station A → To: Clinical Lab
Mode: Run Full Delivery
```

### 시나리오 2: 약품 배송 (단계별)
```
Item: Medicine
From: Main Pharmacy → To: Ward 102
Steps: 
  1. NAV Pickup
  2. DOCK Pickup
  3. PICK
  4. NAV Dropoff
  5. DOCK Dropoff
  6. PLACE
  7. NAV Home
```

### 시나리오 3: 마약류 (보안 중요)
```
Item: Narcotics
From: Main Pharmacy → To: Clinical Lab
Mode: Run Full Delivery
Note: 모든 단계가 로그에 기록됨
```

---

## 📞 도움말

- API 문서: http://localhost:8000/docs
- 상세 가이드: README.md
- 데이터베이스 직접 조회: `sqlite3 hospital_robot.db`

---

## 🎉 Happy Robot Controlling! 🤖
