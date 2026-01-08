import sys
import time
import threading
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QListWidget, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QComboBox, QProgressBar, QGroupBox, QTextEdit, 
                             QFrame, QGridLayout, QMessageBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTime, QSize
from PyQt6.QtGui import QFont, QIcon, QColor

# [ROS2 관련 임포트]
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moma_interfaces.action import RunDelivery
from action_msgs.msg import GoalStatus

# ==========================================
# 1. 환경 설정
# ==========================================
LOCATIONS = [
    "Nurse Station A (Base)", "Ward 101", "Ward 102", "Ward 103", "Ward 104", "Ward 105", 
    "Main Pharmacy (Central)", "Sub Pharmacy", "Clinical Lab (Zone C)", "Central Supply", "Doctor's Office"
]

ITEM_TYPES = {
    "Blood Sample (Emergency)": {"icon": "🩸", "priority": "High", "speed": "Slow", "dest_hint": "Clinical Lab"},
    "General Medicine":         {"icon": "💊", "priority": "Normal", "speed": "Normal", "dest_hint": "Ward 10x"},
    "Narcotics (Secure)":       {"icon": "🔒", "priority": "Critical", "speed": "Fast", "dest_hint": "Doctor's Office"},
    "Surgical Kit":             {"icon": "✂️", "priority": "Normal", "speed": "Normal", "dest_hint": "Operating Room"},
    "Documents/Chart":          {"icon": "📄", "priority": "Low", "speed": "Max", "dest_hint": "Doctor's Office"}
}

PATIENTS = {
    "PT-2401": {"name": "김철수", "ward": "Ward 101", "condition": "Stable"},
    "PT-2402": {"name": "이영희", "ward": "Ward 102", "condition": "Post-Op"},
    "PT-2403": {"name": "박지성", "ward": "Ward 105", "condition": "Critical"},
    "PT-2404": {"name": "최민아", "ward": "Ward 102", "condition": "Check-up"},
}

# ==========================================
# 2. 로봇 워커
# ==========================================
class RobotWorker(QThread):
    log_signal = pyqtSignal(str)          
    state_signal = pyqtSignal(str)        
    progress_signal = pyqtSignal(int)     
    battery_signal = pyqtSignal(int)      
    cam_overlay_signal = pyqtSignal(str)
    task_finished_signal = pyqtSignal()
    retry_request_signal = pyqtSignal(dict, str, str)

    def __init__(self):
        super().__init__()
        self.running = True
        self.queue = []
        self.is_busy = False
        self.battery = 95.0
        self._current_goal_handle = None

        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('ui_action_client')
        self.client = ActionClient(self.node, RunDelivery, 'run_delivery')
        
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def add_task(self, task):
        self.queue.append(task)

    def remove_task_at_index(self, index):
        if 0 <= index < len(self.queue):
            removed = self.queue.pop(index)
            self.log_signal.emit(f"🗑️ Removed task from queue: {removed['item']}")

    def cancel_current_task(self):
        if self.is_busy and self._current_goal_handle:
            self.log_signal.emit("⚠️ Sending CANCEL request to Robot...")
            future = self._current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
        else:
            self.log_signal.emit("ℹ️ No active task to cancel.")

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.log_signal.emit("✅ Cancel request accepted.")
        else:
            self.log_signal.emit("❌ Cancel request failed.")

    def run(self):
        while self.running:
            # 큐 처리 로직
            if not self.is_busy and self.queue:
                current_task = self.queue[0]
                self.execute_task(current_task)
                if self.queue: self.queue.pop(0)
                self.task_finished_signal.emit()
            
            # 배터리 시뮬레이션
            if not self.is_busy:
                time.sleep(1)
                self.battery = max(0, self.battery - 0.01)
                self.battery_signal.emit(int(self.battery))

    # [수정] 즉시 실행 모드 추가 (큐를 거치지 않고 바로 실행)
    def execute_immediate(self, task):
        if self.is_busy:
            self.log_signal.emit("⚠️ Robot is busy! Cannot execute manual step.")
            return
        # 별도 스레드에서 실행하여 UI 프리징 방지
        threading.Thread(target=self.execute_task, args=(task,), daemon=True).start()

    def execute_task(self, task):
        self.is_busy = True
        item_name = task['item']
        pickup = task['from']
        dropoff = task['to']
        # [핵심] task_mode가 없으면 기본값 'ALL' 사용
        mode = task.get('mode', 'ALL') 

        self.log_signal.emit(f"📡 Sending Goal [{mode}]: {item_name}")
        
        goal_msg = RunDelivery.Goal()
        # [주의] Action 파일에 task_mode 필드가 추가되어 있어야 함
        goal_msg.task_mode = mode 
        goal_msg.item_type = item_name
        goal_msg.pickup_loc = pickup
        goal_msg.dropoff_loc = dropoff

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.log_signal.emit("❌ Error: Action Server not available!")
            self.is_busy = False
            return

        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        # 동기 대기 (스레드 내부이므로 UI 안 멈춤)
        while not send_future.done(): time.sleep(0.1)
        
        self._current_goal_handle = send_future.result()
        if not self._current_goal_handle.accepted:
            self.log_signal.emit("❌ Goal Rejected.")
            self.is_busy = False
            self._current_goal_handle = None
            return

        self.log_signal.emit(f"✅ [{mode}] Started...")

        result_future = self._current_goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            # 수동 모드일 때는 배터리 소모 표시 생략 가능

        result_wrapper = result_future.result()
        status = result_wrapper.status
        result = result_wrapper.result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log_signal.emit(f"🎉 Step Completed: {result.message}")
            self.update_state("IDLE", "대기 중")
            self.progress_signal.emit(100 if mode == "ALL" else 0)
            self.cam_overlay_signal.emit("STEP DONE")
        elif status == GoalStatus.STATUS_CANCELED:
            self.log_signal.emit(f"🛑 CANCELED: {result.message}")
            self.update_state("IDLE", "취소됨")
        else:
            self.log_signal.emit(f"💥 Failed: {result.message}")
            self.update_state("ERROR", result.message)
            # 재시도 요청 시, 현재 멈춘 단계(self._last_feedback_state)도 함께 전달
            self.retry_request_signal.emit(task, result.message, self._last_feedback_state)

        self.is_busy = False
        self._current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        state = feedback_msg.feedback.current_state
        self._last_feedback_state = state  # 상태 업데이트 될 때마다 저장
        self.state_signal.emit(state)
        # 로그 과다 출력 방지
        # self.log_signal.emit(f"▶ {state}") 
        
        progress = 0
        if "NAV" in state: progress = 20
        elif "DOCK" in state: progress = 40
        elif "PICK" in state: progress = 50
        elif "PLAC" in state: progress = 90
            
        self.progress_signal.emit(progress)
        self.cam_overlay_signal.emit(state)

    def update_state(self, state, msg):
        self.state_signal.emit(state)

    def stop(self):
        self.running = False
        self.node.destroy_node()

# ==========================================
# 3. 메인 GUI
# ==========================================
class HospitalControlCenter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("H-LOGISTICS CONTROL CENTER")
        self.resize(1600, 1000) # 높이 약간 증가
        self.apply_stylesheet()

        self.robot = RobotWorker()
        self.robot.log_signal.connect(self.add_log)
        self.robot.state_signal.connect(self.update_robot_state)
        self.robot.progress_signal.connect(self.update_progress)
        self.robot.battery_signal.connect(self.update_battery)
        self.robot.cam_overlay_signal.connect(self.update_cam_overlay)
        self.robot.task_finished_signal.connect(self.refresh_queue_list)
        self.robot.retry_request_signal.connect(self.handle_retry_request)
        self.robot.start()

        self.init_ui()

    def apply_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2d30; color: #e0e0e0; }
            QWidget { font-family: 'Segoe UI', 'Malgun Gothic', sans-serif; font-size: 14px; }
            QGroupBox { background-color: #323639; border: 1px solid #454545; border-radius: 8px; margin-top: 25px; font-weight: bold; color: #00adb5; }
            QGroupBox::title { subcontrol-origin: margin; left: 15px; top: 0px; padding: 0 5px; }
            QPushButton { background-color: #4e5254; color: white; border: none; padding: 10px; border-radius: 6px; font-weight: 600; }
            QPushButton:hover { background-color: #5d6163; }
            QPushButton:pressed { background-color: #3b3f41; }
            QPushButton#ActionBtn { background-color: #00adb5; color: #1e1e1e; }
            QPushButton#ActionBtn:hover { background-color: #26c6da; }
            QPushButton#ManualBtn { background-color: #3f51b5; font-size: 13px; padding: 8px; }
            QPushButton#ManualBtn:hover { background-color: #5c6bc0; }
            QPushButton#DeleteBtn { background-color: #d97706; }
            QPushButton#EmergencyBtn { background-color: #c62828; border: 2px solid #ff5252; }
            QTableWidget { background-color: #1e1e1e; color: #ddd; border: 1px solid #3a3a3a; border-radius: 6px; }
            QHeaderView::section { background-color: #3c3f41; color: #fff; padding: 6px; border: none; }
            QComboBox { background-color: #3c3f41; color: white; padding: 8px; border: 1px solid #555; border-radius: 4px; }
            QComboBox QAbstractItemView { background-color: #3c3f41; color: white; selection-background-color: #00adb5; }
            QListWidget { background-color: #1e1e1e; color: #ccc; border: 1px solid #444; border-radius: 6px; }
            QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: 'Consolas', monospace; border: 1px solid #444; }
            QProgressBar { border: 1px solid #444; background-color: #1e1e1e; text-align: center; color: white; }
            QProgressBar::chunk { background-color: #00adb5; }
        """)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # =========================================================
        # [LEFT PANEL]
        # =========================================================
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setFixedWidth(520)
        left_layout.setSpacing(15)

        # 1. Patient List
        grp_patient = QGroupBox(" PATIENT LIST (Zone A)")
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

        # 2. Create Order
        grp_order = QGroupBox(" CREATE WORK ORDER")
        o_layout = QGridLayout()
        
        lbl_item = QLabel("Item Type"); lbl_item.setStyleSheet("color:#aaa; font-weight:bold;")
        self.combo_item = QComboBox(); self.combo_item.addItems(ITEM_TYPES.keys())
        self.combo_item.currentTextChanged.connect(self.update_item_info)

        lbl_from = QLabel("Pickup From"); lbl_from.setStyleSheet("color:#aaa; font-weight:bold;")
        self.combo_from = QComboBox(); self.combo_from.addItems(LOCATIONS)
        self.combo_from.setCurrentText("Main Pharmacy (Central)")

        lbl_to = QLabel("Deliver To"); lbl_to.setStyleSheet("color:#aaa; font-weight:bold;")
        self.combo_to = QComboBox(); self.combo_to.addItems(LOCATIONS)

        self.lbl_item_info = QLabel("Priority: Normal | Speed: Normal")
        self.lbl_item_info.setStyleSheet("background:#333; color:#ccc; padding:4px; border-radius:4px;")
        self.lbl_item_info.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.btn_dispatch = QPushButton("🚀 FULL DISPATCH (Start to Finish)")
        self.btn_dispatch.setObjectName("ActionBtn")
        self.btn_dispatch.setFixedHeight(40)
        self.btn_dispatch.clicked.connect(self.dispatch_task)

        o_layout.addWidget(lbl_item, 0, 0); o_layout.addWidget(self.combo_item, 1, 0, 1, 2)
        o_layout.addWidget(lbl_from, 2, 0); o_layout.addWidget(self.combo_from, 3, 0)
        o_layout.addWidget(lbl_to, 2, 1);   o_layout.addWidget(self.combo_to, 3, 1)
        o_layout.addWidget(self.lbl_item_info, 4, 0, 1, 2)
        o_layout.addWidget(self.btn_dispatch, 5, 0, 1, 2)
        grp_order.setLayout(o_layout)

        # 3. [NEW] Manual Step Control
        grp_step = QGroupBox("🔧 MANUAL STEP CONTROL (Debug/Resume)")
        step_layout = QGridLayout()
        step_layout.setSpacing(8)

        # 버튼 생성 헬퍼
        def create_step_btn(text, mode):
            btn = QPushButton(text)
            btn.setObjectName("ManualBtn")
            btn.clicked.connect(lambda: self.execute_manual_step(mode))
            return btn

        # Pickup Phase
        step_layout.addWidget(QLabel("Pickup Phase:"), 0, 0, 1, 3)
        step_layout.addWidget(create_step_btn("1. Nav to Pickup", "NAV_PICKUP"), 1, 0)
        step_layout.addWidget(create_step_btn("2. Dock (Pick)", "DOCK_PICKUP"), 1, 1)
        step_layout.addWidget(create_step_btn("3. Pick Item", "PICK"), 1, 2)

        # Dropoff Phase
        step_layout.addWidget(QLabel("Dropoff Phase:"), 2, 0, 1, 3)
        step_layout.addWidget(create_step_btn("4. Nav to Dropoff", "NAV_DROPOFF"), 3, 0)
        step_layout.addWidget(create_step_btn("5. Dock (Drop)", "DOCK_DROPOFF"), 3, 1)
        step_layout.addWidget(create_step_btn("6. Place Item", "PLACE"), 3, 2)
        
        # Utils
        btn_nav_home = create_step_btn("🏠 Nav to Home", "NAV_HOME")
        btn_nav_home.setStyleSheet("background-color: #2e7d32; border: 1px solid #4caf50;")
        step_layout.addWidget(btn_nav_home, 4, 0)
        
        btn_home = create_step_btn("🦾 Home Arm", "HOME")
        btn_home.setStyleSheet("background-color: #555; border: 1px solid #777;")
        step_layout.addWidget(btn_home, 4, 1, 1, 2)
        
        

        grp_step.setLayout(step_layout)


        # 4. Queue
        grp_queue = QGroupBox(" TASK QUEUE")
        q_layout = QVBoxLayout()
        self.list_queue = QListWidget()
        self.btn_delete = QPushButton("🗑️ DELETE / CANCEL TASK")
        self.btn_delete.setObjectName("DeleteBtn")
        self.btn_delete.clicked.connect(self.delete_selected_task)
        q_layout.addWidget(self.list_queue)
        q_layout.addWidget(self.btn_delete)
        grp_queue.setLayout(q_layout)

        left_layout.addWidget(grp_patient, 2)
        left_layout.addWidget(grp_order, 0)
        left_layout.addWidget(grp_step, 0) # 수동 제어 패널 추가
        left_layout.addWidget(grp_queue, 2)

        # =========================================================
        # [RIGHT PANEL] (기존과 동일)
        # =========================================================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Status Bar
        status_frame = QFrame()
        status_frame.setStyleSheet("background-color: #323639; border-radius: 8px;")
        status_layout = QHBoxLayout(status_frame)
        self.lbl_state = QLabel("IDLE")
        self.lbl_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_state.setStyleSheet("background-color: #333; color: #fff; font-size: 16px; font-weight: bold; border-radius: 4px; padding: 5px 15px;")
        self.bar_battery = QProgressBar(); self.bar_battery.setValue(95); self.bar_battery.setFixedWidth(120)
        btn_estop = QPushButton("⚠️ ESTOP"); btn_estop.setObjectName("EmergencyBtn"); btn_estop.setFixedWidth(150)
        btn_estop.clicked.connect(self.emergency_stop)
        status_layout.addWidget(QLabel("STATUS:")); status_layout.addWidget(self.lbl_state); status_layout.addStretch()
        status_layout.addWidget(self.bar_battery); status_layout.addWidget(btn_estop)

        # Viewer
        viewer_layout = QHBoxLayout()
        self.frm_camera = QFrame(); self.frm_camera.setStyleSheet("background: #000; border: 2px solid #454545;")
        self.lbl_cam_overlay = QLabel("NO SIGNAL"); self.lbl_cam_overlay.setStyleSheet("color: #0f0; font: 20px Consolas;")
        cam_layout = QVBoxLayout(self.frm_camera); cam_layout.addStretch(); cam_layout.addWidget(self.lbl_cam_overlay); cam_layout.addStretch()
        
        self.frm_map = QFrame(); self.frm_map.setStyleSheet("background: #222; border: 2px solid #454545;")
        viewer_layout.addWidget(self.frm_camera, 6); viewer_layout.addWidget(self.frm_map, 4)

        # Logs
        grp_log = QGroupBox(" SYSTEM LOGS"); l_layout = QVBoxLayout()
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        l_layout.addWidget(self.txt_log); grp_log.setLayout(l_layout)

        self.bar_progress = QProgressBar(); self.bar_progress.setFixedHeight(10); self.bar_progress.setTextVisible(False)

        right_layout.addWidget(status_frame)
        right_layout.addLayout(viewer_layout)
        right_layout.addWidget(grp_log, 1)
        right_layout.addWidget(self.bar_progress)

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
        ward = self.table_patients.item(row, 1).text()
        self.combo_to.setCurrentText(ward)

    def update_item_info(self, item_name):
        info = ITEM_TYPES[item_name]
        self.lbl_item_info.setText(f"Priority: {info['priority']} | Speed: {info['speed']}")
        if "Blood" in item_name:
            self.combo_from.setCurrentText("Nurse Station A (Base)")
            self.combo_to.setCurrentText("Clinical Lab (Zone C)")
        elif "Medicine" in item_name:
            self.combo_from.setCurrentText("Main Pharmacy (Central)")
        elif "Narcotics" in item_name:
            self.combo_from.setCurrentText("Sub Pharmacy")
            self.combo_to.setCurrentText("Doctor's Office")

    # [기존] 전체 시퀀스 실행 (Queue에 추가)
    def dispatch_task(self):
        item = self.combo_item.currentText()
        src = self.combo_from.currentText()
        dst = self.combo_to.currentText()
        
        task_info = {'item': item, 'from': src, 'to': dst, 'mode': 'ALL'} # mode ALL
        self.robot.add_task(task_info)
        self.refresh_queue_list()
        self.add_log(f"Work Order Created: {item}")

    # [신규] 개별 단계 실행 (즉시 실행)
    def execute_manual_step(self, mode):
        # 현재 콤보박스에 선택된 값을 그대로 사용 (이어서 실행하기 위함)
        item = self.combo_item.currentText()
        src = self.combo_from.currentText()
        dst = self.combo_to.currentText()

        # 확인 팝업
        msg = f"Execute step '{mode}'?\nFrom: {src}\nTo: {dst}"
        ret = QMessageBox.question(self, "Manual Execution", msg, 
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        
        if ret == QMessageBox.StandardButton.Yes:
            task_info = {'item': item, 'from': src, 'to': dst, 'mode': mode}
            self.robot.execute_immediate(task_info)
            self.add_log(f"🔧 Manual Step Triggered: {mode}")

    def refresh_queue_list(self):
        self.list_queue.clear()
        for idx, task in enumerate(self.robot.queue):
            status = "▶ RUNNING" if idx == 0 and self.robot.is_busy else "⏳ WAITING"
            self.list_queue.addItem(f"{status} | {task['item']} \n    ({task['from']} -> {task['to']})")

    def delete_selected_task(self):
        row = self.list_queue.currentRow()
        if row < 0:
            QMessageBox.warning(self, "Warning", "Select a task to delete.")
            return
        if row == 0 and self.robot.is_busy:
            ret = QMessageBox.question(self, "Cancel Task", "Current task is running. Cancel?", 
                                       QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if ret == QMessageBox.StandardButton.Yes:
                self.robot.cancel_current_task()
        else:
            self.robot.remove_task_at_index(row)
            self.refresh_queue_list()

    def emergency_stop(self):
        self.add_log("!!! EMERGENCY STOP !!!")
        self.robot.cancel_current_task()
        self.robot.queue.clear()
        self.refresh_queue_list()
        self.lbl_state.setText("ESTOP")
        self.lbl_state.setStyleSheet("background-color: red; color: white;")

    def add_log(self, msg):
        t = QTime.currentTime().toString("HH:mm:ss")
        self.txt_log.append(f"[{t}] {msg}")
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def update_robot_state(self, state):
        self.lbl_state.setText(state)
        style = "font-size: 16px; font-weight: bold; border-radius: 4px; padding: 5px 15px; color: white;"
        if "IDLE" in state: self.lbl_state.setStyleSheet(f"background:#444; {style}")
        elif "NAV" in state: self.lbl_state.setStyleSheet(f"background:#0078d7; {style}")
        elif "SCAN" in state or "PICK" in state: self.lbl_state.setStyleSheet(f"background:#d97706; {style}")
        elif "DOCK" in state: self.lbl_state.setStyleSheet(f"background:#059669; {style}")
        else: self.lbl_state.setStyleSheet(f"background:#333; {style}")

    def update_progress(self, val): self.bar_progress.setValue(val)
    def update_battery(self, val): self.bar_battery.setValue(val)
    def update_cam_overlay(self, text): self.lbl_cam_overlay.setText(text)
    
    # [수정] 실패한 단계부터 이어하기 로직 적용
    def handle_retry_request(self, failed_task, error_msg, last_state):
        """실패한 태스크에 대해 재시도 여부를 사용자에게 확인 (이어하기 기능 포함)"""
        mode = failed_task.get('mode', 'ALL')
        item = failed_task.get('item', 'Unknown')
        
        # 1. 실패한 상태(last_state)를 기반으로 재시작할 단계 매핑
        resume_mode = "ALL" # 기본값
        if "NAVIGATING TO PICKUP" in last_state: resume_mode = "NAV_PICKUP_CONT"
        elif "DOCKING AT PICKUP" in last_state:  resume_mode = "DOCK_PICKUP_CONT"
        elif "PICKING" in last_state:            resume_mode = "PICK_CONT" # SCANNING & PICKING
        elif "NAVIGATING TO DROPOFF" in last_state: resume_mode = "NAV_DROPOFF_CONT"
        elif "DOCKING AT DROPOFF" in last_state: resume_mode = "DOCK_DROPOFF_CONT"
        elif "PLACING" in last_state:            resume_mode = "PLACE_CONT"
        elif "RETURNING" in last_state:          resume_mode = "NAV_HOME_CONT"
        
        msg = f"작업 실패!\n\n모드: {mode}\n중단 지점: {last_state}\n\n'예'를 누르면 [{resume_mode}] 모드로 이어서 진행합니다."
        
        ret = QMessageBox.question(
            self, "⚠️ Task Failed - Resume?", msg,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if ret == QMessageBox.StandardButton.Yes:
            # 이어하기용 새로운 태스크 생성
            new_task = failed_task.copy()
            new_task['mode'] = resume_mode
            
            self.add_log(f"🔄 Resuming Task: {resume_mode}...")
            self.robot.execute_immediate(new_task)
        else:
            self.add_log(f"❌ Retry skipped by user.")

def main(args=None):
    app = QApplication(sys.argv)
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    window = HospitalControlCenter()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()