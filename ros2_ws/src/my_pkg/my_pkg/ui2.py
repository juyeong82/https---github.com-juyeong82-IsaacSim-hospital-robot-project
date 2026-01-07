import sys
import time
import random
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QListWidget, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QComboBox, QProgressBar, QGroupBox, QTextEdit, 
                             QSplitter, QMessageBox, QFrame, QGridLayout)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QTime
from PyQt6.QtGui import QFont, QColor, QPixmap, QPainter, QPen

# ==========================================
# 1. 환경 설정 (우리가 설계한 11개 방 & 에셋)
# ==========================================

# 방 목록 (11 Rooms)
LOCATIONS = [
    "Nurse Station A (Base)", 
    "Ward 101", "Ward 102", "Ward 103", "Ward 104", "Ward 105", 
    "Main Pharmacy (Central)", "Sub Pharmacy", 
    "Clinical Lab (Zone C)", "Central Supply", "Doctor's Office"
]

# 이송 물품 타입 (시나리오별 속성)
ITEM_TYPES = {
    "Blood Sample (Emergency)": {"icon": "🩸", "priority": "High", "speed": "Slow (Safety Mode)", "dest_hint": "Clinical Lab"},
    "General Medicine":         {"icon": "💊", "priority": "Normal", "speed": "Normal", "dest_hint": "Ward 10x"},
    "Narcotics (Secure)":       {"icon": "🔒", "priority": "Critical", "speed": "Fast", "dest_hint": "Doctor's Office"},
    "Surgical Kit":             {"icon": "✂️", "priority": "Normal", "speed": "Normal", "dest_hint": "Operating Room"},
    "Documents/Chart":          {"icon": "📄", "priority": "Low", "speed": "Max", "dest_hint": "Doctor's Office"}
}

# 가상의 환자 데이터 (병실 매핑)
PATIENTS = {
    "PT-2401": {"name": "김철수", "ward": "Ward 101", "condition": "Stable"},
    "PT-2402": {"name": "이영희", "ward": "Ward 102", "condition": "Post-Op"},
    "PT-2403": {"name": "박지성", "ward": "Ward 105", "condition": "Critical"},
    "PT-2404": {"name": "최민아", "ward": "Ward 103", "condition": "Check-up"},
}

# ==========================================
# 2. 로봇 시뮬레이션 워커 (ROS2 연동용)
# ==========================================
class RobotWorker(QThread):
    # UI 업데이트용 신호
    log_signal = pyqtSignal(str)          # 로그 텍스트
    state_signal = pyqtSignal(str)        # 현재 상태 (NAV, DOCK, GRASP...)
    progress_signal = pyqtSignal(int)     # 작업 진행률
    battery_signal = pyqtSignal(int)      # 배터리
    cam_overlay_signal = pyqtSignal(str)  # 카메라 오버레이 텍스트 (ArUco ID 등)

    def __init__(self):
        super().__init__()
        self.running = True
        self.queue = []
        self.is_busy = False
        self.battery = 95

    def add_task(self, task):
        self.queue.append(task)

    def run(self):
        while self.running:
            if not self.is_busy and self.queue:
                current_task = self.queue.pop(0)
                self.execute_task(current_task)
            
            # Idle 상태일 때 배터리 관리 등
            if not self.is_busy:
                time.sleep(1)
                self.battery = max(0, self.battery - 0.01) # 대기 중 소모
                self.battery_signal.emit(int(self.battery))

    def execute_task(self, task):
        self.is_busy = True
        item_name = task['item'].split('(')[0].strip()
        
        # 1. 출발지로 이동
        self.update_state("NAVIGATING", f"이동 중: {task['from']}")
        self.simulate_process(3, "Moving to Pickup Zone...")

        # 2. 물체 인식 (Vision)
        self.update_state("VISION_SCAN", "ArUco 마커 탐색 중...")
        self.cam_overlay_signal.emit("SCANNING...")
        time.sleep(1)
        self.cam_overlay_signal.emit(f"DETECTED: {item_name}\nID: {random.randint(10,99)}\nDIST: 0.45m")
        self.log_signal.emit(f"[Vision] Target Detected: {item_name}")
        time.sleep(1.5)

        # 3. 파지 (Manipulation)
        self.update_state("MANIPULATION", "RMPFlow: Grasping Target")
        self.simulate_process(2, "Gripper Closing...")
        self.cam_overlay_signal.emit("GRASPED")

        # 4. 목적지로 이동 (Nav2)
        speed_mode = ITEM_TYPES[task['item']]['speed']
        self.update_state("NAVIGATING", f"이동 중: {task['to']} ({speed_mode})")
        self.log_signal.emit(f"[Nav2] Path planned to {task['to']}. Mode: {speed_mode}")
        
        # 이동 중 장애물 회피 시뮬레이션 (랜덤)
        if random.random() < 0.3:
            self.log_signal.emit("[Nav2] ⚠️ Obstacle Detected! Re-planning path...")
            time.sleep(1)
        
        self.simulate_process(4, "Autonomous Driving...")

        # 5. 정밀 도킹 (Docking)
        self.update_state("DOCKING", "AprilTag 정밀 접근 중...")
        self.cam_overlay_signal.emit("TAG DETECTED: DOCK_01")
        time.sleep(1.5)
        self.cam_overlay_signal.emit("ALIGNING...")
        time.sleep(1)

        # 6. 하역 & 완료
        self.update_state("PLACING", "물품 하역 중...")
        time.sleep(1)
        self.cam_overlay_signal.emit("RELEASED")
        self.log_signal.emit(f"[Success] Delivered {item_name} to {task['to']}")
        
        self.is_busy = False
        self.update_state("IDLE", "대기 중")
        self.cam_overlay_signal.emit("NO SIGNAL")
        self.progress_signal.emit(100)

    def update_state(self, state, msg):
        self.state_signal.emit(state)
        self.log_signal.emit(f"[{state}] {msg}")

    def simulate_process(self, duration, log_msg):
        steps = 10
        for i in range(steps):
            time.sleep(duration / steps)
            self.progress_signal.emit(int((i+1) * 10))
            self.battery -= 0.1
            self.battery_signal.emit(int(self.battery))

    def stop(self):
        self.running = False

# ==========================================
# 3. 메인 관제 센터 GUI
# ==========================================
class HospitalControlCenter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("H-LOGISTICS CONTROL CENTER (v2.0)")
        self.resize(1600, 900)
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; color: #ffffff; }
            QWidget { font-family: 'Segoe UI', sans-serif; }
            QGroupBox { 
                border: 1px solid #444; border-radius: 6px; margin-top: 10px; 
                color: #ddd; font-weight: bold; background-color: #262626;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #eee; }
            QPushButton { 
                background-color: #0078d7; color: white; border: none; padding: 8px; border-radius: 4px; font-weight: bold;
            }
            QPushButton:hover { background-color: #0063b1; }
            QPushButton:disabled { background-color: #555; color: #aaa; }
            QPushButton#EmergencyBtn { background-color: #dc2626; }
            QPushButton#EmergencyBtn:hover { background-color: #b91c1c; }
            
            QTableWidget { background-color: #333; gridline-color: #555; color: white; border: none; }
            QHeaderView::section { background-color: #444; color: white; padding: 4px; border: 1px solid #555; }
            
            QComboBox { background-color: #333; color: white; padding: 5px; border: 1px solid #555; }
            QListWidget { background-color: #333; color: white; border: 1px solid #555; }
            QTextEdit { background-color: #111; color: #0f0; font-family: Consolas; border: 1px solid #444; }
            QProgressBar { border: 1px solid #555; text-align: center; color: white; }
            QProgressBar::chunk { background-color: #0078d7; }
        """)

        # 로봇 워커 초기화
        self.robot = RobotWorker()
        self.robot.log_signal.connect(self.add_log)
        self.robot.state_signal.connect(self.update_robot_state)
        self.robot.progress_signal.connect(self.update_progress)
        self.robot.battery_signal.connect(self.update_battery)
        self.robot.cam_overlay_signal.connect(self.update_cam_overlay)
        self.robot.start()

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(15)

        # ---------------------------------------------------------
        # [LEFT PANEL] 작업 지시 & 데이터 (Task Dispatcher)
        # ---------------------------------------------------------
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setFixedWidth(500)

        # 1. 환자 정보 테이블
        grp_patient = QGroupBox("🏥 PATIENT LIST (Zone A)")
        p_layout = QVBoxLayout()
        self.table_patients = QTableWidget()
        self.table_patients.setColumnCount(3)
        self.table_patients.setHorizontalHeaderLabels(["Name", "Ward", "Status"])
        self.table_patients.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_patients.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.load_patient_data()
        self.table_patients.cellClicked.connect(self.auto_fill_destination)
        p_layout.addWidget(self.table_patients)
        grp_patient.setLayout(p_layout)

        # 2. 작업 생성 (Work Order)
        grp_order = QGroupBox("📋 CREATE WORK ORDER")
        o_layout = QGridLayout()
        
        # Item Type
        o_layout.addWidget(QLabel("Item Type:"), 0, 0)
        self.combo_item = QComboBox()
        self.combo_item.addItems(ITEM_TYPES.keys())
        self.combo_item.currentTextChanged.connect(self.update_item_info)
        o_layout.addWidget(self.combo_item, 0, 1)

        # From (Pickup)
        o_layout.addWidget(QLabel("From (Pickup):"), 1, 0)
        self.combo_from = QComboBox()
        self.combo_from.addItems(LOCATIONS)
        self.combo_from.setCurrentText("Main Pharmacy (Central)")
        o_layout.addWidget(self.combo_from, 1, 1)

        # To (Drop-off)
        o_layout.addWidget(QLabel("To (Dest):"), 2, 0)
        self.combo_to = QComboBox()
        self.combo_to.addItems(LOCATIONS)
        o_layout.addWidget(self.combo_to, 2, 1)

        # Info Label (속도 등 표시)
        self.lbl_item_info = QLabel("Priority: Normal | Speed: Normal")
        self.lbl_item_info.setStyleSheet("color: #aaa; font-size: 11px;")
        o_layout.addWidget(self.lbl_item_info, 3, 0, 1, 2)

        # Dispatch Button
        self.btn_dispatch = QPushButton("🚀 DISPATCH ROBOT")
        self.btn_dispatch.setFixedHeight(40)
        self.btn_dispatch.clicked.connect(self.dispatch_task)
        o_layout.addWidget(self.btn_dispatch, 4, 0, 1, 2)

        grp_order.setLayout(o_layout)

        # 3. 작업 대기열 (Queue)
        grp_queue = QGroupBox("⏳ TASK QUEUE")
        q_layout = QVBoxLayout()
        self.list_queue = QListWidget()
        q_layout.addWidget(self.list_queue)
        grp_queue.setLayout(q_layout)

        left_layout.addWidget(grp_patient, 1)
        left_layout.addWidget(grp_order, 0)
        left_layout.addWidget(grp_queue, 1)

        # ---------------------------------------------------------
        # [RIGHT PANEL] 관제 모니터링 (Monitoring Center)
        # ---------------------------------------------------------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # 1. 상단 상태 바
        status_layout = QHBoxLayout()
        
        # Robot State Box
        self.lbl_state = QLabel("IDLE")
        self.lbl_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_state.setStyleSheet("background-color: #333; color: #0f0; font-size: 18px; font-weight: bold; border-radius: 4px; padding: 10px;")
        self.lbl_state.setFixedWidth(150)
        
        # Battery
        self.bar_battery = QProgressBar()
        self.bar_battery.setValue(95)
        self.bar_battery.setFormat("BAT: %p%")
        self.bar_battery.setFixedWidth(150)
        
        # Emergency Stop
        btn_estop = QPushButton("⚠️ EMERGENCY STOP")
        btn_estop.setObjectName("EmergencyBtn")
        btn_estop.setFixedWidth(180)
        btn_estop.clicked.connect(self.emergency_stop)

        status_layout.addWidget(QLabel("ROBOT STATE:"))
        status_layout.addWidget(self.lbl_state)
        status_layout.addStretch()
        status_layout.addWidget(self.bar_battery)
        status_layout.addWidget(btn_estop)

        # 2. 메인 뷰어 (카메라 + 맵) - 시뮬레이션용 플레이스홀더
        viewer_layout = QHBoxLayout()
        
        # Camera View (Simulated)
        self.frm_camera = QFrame()
        self.frm_camera.setStyleSheet("background-color: #000; border: 2px solid #555;")
        self.frm_camera.setMinimumHeight(400)
        cam_layout = QVBoxLayout(self.frm_camera)
        
        lbl_cam_title = QLabel("🎥 ROBOT EYE (RGB-D)")
        lbl_cam_title.setStyleSheet("color: white; font-weight: bold; background: rgba(0,0,0,0.5);")
        
        self.lbl_cam_overlay = QLabel("NO SIGNAL")
        self.lbl_cam_overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_cam_overlay.setStyleSheet("color: #0f0; font-family: Consolas; font-size: 20px;")
        
        cam_layout.addWidget(lbl_cam_title, 0, Qt.AlignmentFlag.AlignTop)
        cam_layout.addStretch()
        cam_layout.addWidget(self.lbl_cam_overlay)
        cam_layout.addStretch()

        # Map View (Simulated)
        self.frm_map = QFrame()
        self.frm_map.setStyleSheet("background-color: #222; border: 2px solid #555;")
        map_layout = QVBoxLayout(self.frm_map)
        lbl_map_title = QLabel("🗺️ NAV2 COSTMAP")
        lbl_map_title.setStyleSheet("color: white; font-weight: bold;")
        lbl_map_placeholder = QLabel("[Live Map Stream]")
        lbl_map_placeholder.setStyleSheet("color: #555;")
        lbl_map_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        map_layout.addWidget(lbl_map_title)
        map_layout.addWidget(lbl_map_placeholder)

        viewer_layout.addWidget(self.frm_camera, 2)
        viewer_layout.addWidget(self.frm_map, 1)

        # 3. 시스템 로그
        grp_log = QGroupBox("📟 SYSTEM LOGS")
        l_layout = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        l_layout.addWidget(self.txt_log)
        grp_log.setLayout(l_layout)

        # 4. 작업 진행률
        self.bar_progress = QProgressBar()
        self.bar_progress.setValue(0)
        self.bar_progress.setStyleSheet("QProgressBar {height: 20px;}")

        right_layout.addLayout(status_layout)
        right_layout.addSpacing(10)
        right_layout.addLayout(viewer_layout)
        right_layout.addWidget(grp_log, 1)
        right_layout.addWidget(self.bar_progress)

        # Main Layout 합치기
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel)

    # ==========================================
    # Logic Methods
    # ==========================================
    def load_patient_data(self):
        self.table_patients.setRowCount(len(PATIENTS))
        for i, (pid, info) in enumerate(PATIENTS.items()):
            self.table_patients.setItem(i, 0, QTableWidgetItem(info['name']))
            self.table_patients.setItem(i, 1, QTableWidgetItem(info['ward']))
            self.table_patients.setItem(i, 2, QTableWidgetItem(info['condition']))

    def auto_fill_destination(self, row, col):
        # 환자 선택 시 도착지를 해당 병실로 자동 설정
        ward = self.table_patients.item(row, 1).text()
        self.combo_to.setCurrentText(ward)
        self.add_log(f"Destination set to {ward}")

    def update_item_info(self, item_name):
        info = ITEM_TYPES[item_name]
        self.lbl_item_info.setText(f"Priority: {info['priority']} | Speed: {info['speed']}")
        
        # 추천 출발/도착지 자동 세팅 (편의성)
        if "Blood" in item_name:
            self.combo_from.setCurrentText("Nurse Station A (Base)")
            self.combo_to.setCurrentText("Clinical Lab (Zone C)")
        elif "Medicine" in item_name:
            self.combo_from.setCurrentText("Main Pharmacy (Central)")
        elif "Narcotics" in item_name:
            self.combo_from.setCurrentText("Sub Pharmacy")
            self.combo_to.setCurrentText("Doctor's Office")

    def dispatch_task(self):
        item = self.combo_item.currentText()
        src = self.combo_from.currentText()
        dst = self.combo_to.currentText()
        
        task_info = f"{ITEM_TYPES[item]['icon']} {item.split('(')[0]} : {src} ➔ {dst}"
        self.list_queue.addItem(task_info)
        
        # 로봇 스레드로 작업 전달
        self.robot.add_task({
            'item': item,
            'from': src,
            'to': dst
        })
        self.add_log(f"Work Order Created: {task_info}")

    def emergency_stop(self):
        self.robot.stop()
        self.lbl_state.setText("ESTOP")
        self.lbl_state.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 20px;")
        self.add_log("!!! EMERGENCY STOP TRIGGERED !!!")
        QMessageBox.critical(self, "EMERGENCY", "ROBOT STOPPED IMMEDIATELY!")

    # ==========================================
    # Slots for Robot Signals
    # ==========================================
    def add_log(self, msg):
        t = QTime.currentTime().toString("HH:mm:ss")
        self.txt_log.append(f"[{t}] {msg}")
        # 자동 스크롤
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def update_robot_state(self, state):
        self.lbl_state.setText(state)
        # 상태별 색상 변경
        if state == "IDLE": color = "#555"
        elif state == "NAVIGATING": color = "#0078d7"
        elif state == "VISION_SCAN": color = "#d97706" # Yellow/Orange
        elif state == "MANIPULATION": color = "#9333ea" # Purple
        elif state == "DOCKING": color = "#16a34a" # Green
        else: color = "#333"
        
        self.lbl_state.setStyleSheet(f"background-color: {color}; color: white; font-size: 18px; font-weight: bold; border-radius: 4px; padding: 10px;")

    def update_progress(self, val):
        self.bar_progress.setValue(val)

    def update_battery(self, val):
        self.bar_battery.setValue(val)
        if val < 20:
            self.bar_battery.setStyleSheet("QProgressBar::chunk { background-color: red; }")
        else:
            self.bar_battery.setStyleSheet("QProgressBar::chunk { background-color: #0078d7; }")

    def update_cam_overlay(self, text):
        self.lbl_cam_overlay.setText(text)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 기본 폰트 설정
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    
    window = HospitalControlCenter()
    window.show()
    sys.exit(app.exec())