import sys
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QListWidget, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QComboBox, QProgressBar, QGroupBox, QTextEdit, 
                             QSplitter, QMessageBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QIcon, QColor

# ==========================================
# 1. 데이터 모델 (기존 데이터 유지)
# ==========================================
PATIENTS = {
    "P2025001": {"name": "홍길동", "ward": "301호 (내과)", "notes": "낙상 주의"},
    "P2025002": {"name": "김영희", "ward": "502호 (외과)", "notes": "수술 후 회복"},
    "P2025003": {"name": "이민수", "ward": "ER-01 (응급)", "notes": "응급 검사"},
}

SAMPLES = [
    {"id": "SMP-001", "pid": "P2025001", "type": "Blood (EDTA)", "status": "Ready", "dest": "Central Lab"},
    {"id": "SMP-002", "pid": "P2025001", "type": "Urine", "status": "Ready", "dest": "Microbio Lab"},
    {"id": "SMP-003", "pid": "P2025002", "type": "Tissue", "status": "Ready", "dest": "Pathology"},
]

LOCATIONS = ["Station A", "Station B", "Ward 301", "Ward 502", "ER", "Central Lab", "Pathology"]

# ==========================================
# 2. ROS 2 통신 시뮬레이션 스레드
# (추후 실제 rclpy 노드로 교체될 부분)
# ==========================================
class RosWorker(QThread):
    # UI로 보낼 신호들
    status_update = pyqtSignal(str)   # 로봇 상태 메시지
    battery_update = pyqtSignal(int)  # 배터리 잔량
    location_update = pyqtSignal(str) # 현재 위치
    task_finished = pyqtSignal(str)   # 작업 완료 알림

    def __init__(self):
        super().__init__()
        self.running = True
        self.task_queue = []      # 작업을 쌓아두는 리스트
        self.last_status = ""     # ★ 핵심: 중복 로그 방지용 변수

    def run(self):
        battery = 100
        while self.running:
            # 1. 수행할 작업이 있는지 확인
            if self.task_queue:
                self.execute_pending_tasks()
                # 작업이 끝나면 마지막 상태 초기화 (IDLE 로그가 다시 찍히도록)
                self.last_status = "FINISHED"
            
            else:
                # 2. 할 일이 없을 때 (IDLE 상태)
                # ★ 여기가 수정됨: 이전 상태가 'IDLE'이 아닐 때만 로그 전송
                if self.last_status != "IDLE":
                    self.status_update.emit("IDLE - 대기 중")
                    self.last_status = "IDLE"
                
                # 배터리 자연 소모 (로그 없이 조용히 업데이트)
                battery = battery - 1 if battery > 0 else 100
                self.battery_update.emit(battery)
                
                time.sleep(1) # 1초 대기

    def send_task(self, task_list):
        """UI에서 호출: 작업 리스트를 큐에 추가만 하고 바로 복귀 (Non-blocking)"""
        self.task_queue.extend(task_list)

    def execute_pending_tasks(self):
        """실제 작업 수행 (스레드 내부에서 실행됨)"""
        # 큐에 있는 모든 작업을 가져옴
        current_tasks = list(self.task_queue)
        self.task_queue.clear()

        for task in current_tasks:
            # [이동]
            self.report_status(f"이동 중 -> {task['pickup']} (수거)")
            self.location_update.emit("이동 중...")
            time.sleep(2)
            
            # [파지]
            self.report_status(f"작업 중 -> 검체 파지 ({task['sample_id']})")
            self.location_update.emit(task['pickup'])
            time.sleep(2)
            
            # [이동]
            self.report_status(f"이동 중 -> {task['dest']} (제출)")
            self.location_update.emit("이동 중...")
            time.sleep(2)
            
            # [하역]
            self.report_status(f"작업 중 -> 검체 하역")
            self.location_update.emit(task['dest'])
            time.sleep(1)
            
        # 복귀 및 완료
        self.report_status("충전 스테이션으로 복귀 중...")
        time.sleep(2)
        self.location_update.emit("Charging Station")
        self.report_status("IDLE - 충전 중")
        self.task_finished.emit("모든 작업이 완료되었습니다.")

    def report_status(self, msg):
        """상태 전송 헬퍼 함수"""
        self.status_update.emit(msg)
        self.last_status = msg # 현재 상태 기록

# ==========================================
# 3. 메인 윈도우 (GUI)
# ==========================================
class HospitalRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MoMa Control System - Nurse Station")
        self.resize(1400, 800)
        self.setStyleSheet("""
            QMainWindow { background-color: #f0f2f5; }
            QGroupBox { font-weight: bold; border: 1px solid #dcdcdc; border-radius: 5px; margin-top: 10px; background-color: white; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; color: #333; }
            QPushButton { background-color: #0078d7; color: white; border-radius: 4px; padding: 6px; font-weight: bold; }
            QPushButton:hover { background-color: #005a9e; }
            QPushButton:disabled { background-color: #cccccc; }
            QTableWidget { gridline-color: #e0e0e0; selection-background-color: #cce8ff; selection-color: black; }
            QProgressBar { border: 1px solid #bbb; border-radius: 4px; text-align: center; }
            QProgressBar::chunk { background-color: #00cc66; }
        """)

        # 데이터 저장소
        self.task_queue = []

        # UI 초기화
        self.init_ui()
        
        # ROS 스레드 시작
        self.ros_thread = RosWorker()
        self.ros_thread.status_update.connect(self.update_status_label)
        self.ros_thread.battery_update.connect(self.update_battery)
        self.ros_thread.location_update.connect(self.update_location)
        self.ros_thread.task_finished.connect(self.on_task_finished)
        self.ros_thread.start()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # [1] 상단 상태 바 (Robot Status Bar)
        status_bar = QGroupBox("Robot Status")
        status_layout = QHBoxLayout()
        
        self.lbl_status = QLabel("State: IDLE")
        self.lbl_status.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.lbl_status.setStyleSheet("color: #d9534f;") # Red color for status
        
        self.lbl_location = QLabel("Loc: Charging Station")
        self.lbl_location.setFont(QFont("Arial", 11))
        
        self.progress_battery = QProgressBar()
        self.progress_battery.setValue(100)
        self.progress_battery.setFixedWidth(200)
        self.progress_battery.setFormat("Battery: %p%")

        btn_stop = QPushButton("STOP (Emergency)")
        btn_stop.setStyleSheet("background-color: #dc3545; height: 30px;")
        btn_stop.clicked.connect(self.emergency_stop)

        status_layout.addWidget(self.lbl_status)
        status_layout.addStretch(1)
        status_layout.addWidget(self.lbl_location)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.progress_battery)
        status_layout.addSpacing(20)
        status_layout.addWidget(btn_stop)
        status_bar.setLayout(status_layout)
        
        main_layout.addWidget(status_bar)

        # [2] 메인 컨텐츠 (Splitter 사용)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # --- 좌측: 환자 및 검체 선택 ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # 환자 목록 그룹
        grp_patient = QGroupBox("1. Select Patient")
        vbox_pat = QVBoxLayout()
        self.table_patients = QTableWidget()
        self.table_patients.setColumnCount(3)
        self.table_patients.setHorizontalHeaderLabels(["ID", "Name", "Ward"])
        self.table_patients.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_patients.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table_patients.cellClicked.connect(self.on_patient_selected)
        vbox_pat.addWidget(self.table_patients)
        grp_patient.setLayout(vbox_pat)
        
        # 검체 목록 그룹
        grp_sample = QGroupBox("2. Select Sample")
        vbox_samp = QVBoxLayout()
        self.table_samples = QTableWidget()
        self.table_samples.setColumnCount(4)
        self.table_samples.setHorizontalHeaderLabels(["ID", "Type", "Status", "Destination"])
        self.table_samples.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_samples.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        vbox_samp.addWidget(self.table_samples)
        
        # 추가 버튼 영역
        hbox_add = QHBoxLayout()
        self.combo_pickup = QComboBox()
        self.combo_pickup.addItems(LOCATIONS) # 출발지 수동 선택 가능
        btn_add_queue = QPushButton("Add to Queue ↓")
        btn_add_queue.clicked.connect(self.add_to_queue)
        
        hbox_add.addWidget(QLabel("Pickup:"))
        hbox_add.addWidget(self.combo_pickup)
        hbox_add.addWidget(btn_add_queue)
        vbox_samp.addLayout(hbox_add)
        
        grp_sample.setLayout(vbox_samp)

        left_layout.addWidget(grp_patient)
        left_layout.addWidget(grp_sample)
        
        # --- 우측: 작업 큐 및 실행 ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        grp_queue = QGroupBox("3. Task Queue")
        vbox_queue = QVBoxLayout()
        self.list_queue = QListWidget()
        vbox_queue.addWidget(self.list_queue)
        
        hbox_queue_ctrl = QHBoxLayout()
        btn_remove = QPushButton("Remove Selected")
        btn_remove.setStyleSheet("background-color: #6c757d;")
        btn_remove.clicked.connect(self.remove_from_queue)
        
        self.btn_run = QPushButton("START ROBOT ▶")
        self.btn_run.setStyleSheet("background-color: #28a745; font-size: 14px; height: 40px;")
        self.btn_run.clicked.connect(self.run_robot)
        
        hbox_queue_ctrl.addWidget(btn_remove)
        hbox_queue_ctrl.addWidget(self.btn_run)
        vbox_queue.addLayout(hbox_queue_ctrl)
        grp_queue.setLayout(vbox_queue)

        # 로그 창
        grp_log = QGroupBox("System Log")
        vbox_log = QVBoxLayout()
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setStyleSheet("background-color: #f8f9fa; font-family: Consolas;")
        vbox_log.addWidget(self.text_log)
        grp_log.setLayout(vbox_log)

        right_layout.addWidget(grp_queue, stretch=2)
        right_layout.addWidget(grp_log, stretch=1)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([600, 600]) # 초기 비율

        main_layout.addWidget(splitter)

        # 데이터 채우기
        self.load_patients()

    # ==========================
    # 로직 함수들
    # ==========================
    def load_patients(self):
        self.table_patients.setRowCount(len(PATIENTS))
        for r, (pid, info) in enumerate(PATIENTS.items()):
            self.table_patients.setItem(r, 0, QTableWidgetItem(pid))
            self.table_patients.setItem(r, 1, QTableWidgetItem(info['name']))
            self.table_patients.setItem(r, 2, QTableWidgetItem(info['ward']))

    def on_patient_selected(self, row, col):
        pid = self.table_patients.item(row, 0).text()
        
        # 해당 환자의 샘플 필터링
        filtered_samples = [s for s in SAMPLES if s['pid'] == pid]
        
        self.table_samples.setRowCount(len(filtered_samples))
        for r, s in enumerate(filtered_samples):
            self.table_samples.setItem(r, 0, QTableWidgetItem(s['id']))
            self.table_samples.setItem(r, 1, QTableWidgetItem(s['type']))
            self.table_samples.setItem(r, 2, QTableWidgetItem(s['status']))
            self.table_samples.setItem(r, 3, QTableWidgetItem(s['dest']))
            
        # 콤보박스 자동 설정 (환자 병실로)
        ward = PATIENTS[pid]['ward']
        # 예시 로직: ward 문자열이 LOCATIONS에 있으면 자동 선택
        # 실제로는 매핑 테이블 필요
        if "301" in ward: self.combo_pickup.setCurrentText("Ward 301")
        elif "502" in ward: self.combo_pickup.setCurrentText("Ward 502")
        elif "ER" in ward: self.combo_pickup.setCurrentText("ER")

    def add_to_queue(self):
        current_row = self.table_samples.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "Warning", "Please select a sample first.")
            return
            
        sample_id = self.table_samples.item(current_row, 0).text()
        dest = self.table_samples.item(current_row, 3).text()
        pickup = self.combo_pickup.currentText()
        
        task_str = f"[Pickup: {pickup}] -> [Sample: {sample_id}] -> [Dest: {dest}]"
        
        # 중복 체크
        items = [self.list_queue.item(i).text() for i in range(self.list_queue.count())]
        if task_str in items:
            return

        self.list_queue.addItem(task_str)
        self.task_queue.append({
            "sample_id": sample_id,
            "pickup": pickup,
            "dest": dest
        })
        self.log(f"Added to queue: {sample_id}")

    def remove_from_queue(self):
        row = self.list_queue.currentRow()
        if row >= 0:
            removed = self.list_queue.takeItem(row)
            del self.task_queue[row]
            self.log(f"Removed from queue: {removed.text()}")

    def run_robot(self):
        if not self.task_queue:
            QMessageBox.information(self, "Info", "Queue is empty.")
            return
            
        self.btn_run.setEnabled(False)
        self.log(">>> Mission Start sent to Robot.")
        
        # ROS 스레드에 작업 전달
        self.ros_thread.send_task(self.task_queue)

    def emergency_stop(self):
        self.log("!!! EMERGENCY STOP TRIGGERED !!!")
        # 실제 구현에서는 ROS Service로 Cancel Request 전송 필요
        # self.ros_thread.stop_robot()
        self.lbl_status.setText("State: ESTOP")

    # ==========================
    # ROS 스레드 슬롯 (업데이트)
    # ==========================
    def update_status_label(self, msg):
        self.lbl_status.setText(f"State: {msg}")
        self.log(f"[Status] {msg}")

    def update_battery(self, val):
        self.progress_battery.setValue(val)
        # 배터리 색상 변경 로직
        if val < 20: self.progress_battery.setStyleSheet("QProgressBar::chunk { background-color: red; }")
        else: self.progress_battery.setStyleSheet("QProgressBar::chunk { background-color: #00cc66; }")

    def update_location(self, loc):
        self.lbl_location.setText(f"Loc: {loc}")

    def on_task_finished(self, msg):
        self.btn_run.setEnabled(True)
        self.list_queue.clear()
        self.task_queue.clear()
        QMessageBox.information(self, "Done", msg)

    def log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.text_log.append(f"[{timestamp}] {msg}")
        self.text_log.verticalScrollBar().setValue(self.text_log.verticalScrollBar().maximum())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HospitalRobotGUI()
    window.show()
    sys.exit(app.exec())