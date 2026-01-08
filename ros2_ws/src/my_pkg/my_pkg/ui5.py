import sys
import threading
import time
import math
import numpy as np
import cv2
import os

# [중요] OpenCV와 Qt 플러그인 충돌 방지 (가장 먼저 실행)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

# PyQt6 Imports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QComboBox, QTextEdit, QProgressBar, QGroupBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QImage, QPixmap, QColor

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# Custom Interfaces
from moma_interfaces.action import RunDelivery
from moma_interfaces.msg import MarkerArray

# DB Import (기존 파일 그대로 사용 가능)
from database_manager import HospitalDB

class RosWorker(QObject):
    """ROS Spin을 위한 워커 (스레드 안전성 확보)"""
    finished = pyqtSignal()

    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS Spin Error: {e}")
        finally:
            self.finished.emit()

class HospitalControlNode(Node):
    def __init__(self):
        super().__init__('hospital_ui_node')
        
        # Action Client
        self._action_client = ActionClient(self, RunDelivery, 'run_delivery')
        
        # Subscribers
        self.create_subscription(Image, '/front_camera/rgb', self.front_cam_cb, 10)
        self.create_subscription(Image, '/left_camera/rgb', self.left_cam_cb, 10)
        self.create_subscription(Image, '/right_camera/rgb', self.right_cam_cb, 10)
        
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.dock_pose_cb, 10)
        self.create_subscription(MarkerArray, '/vision/left_markers', self.left_marker_cb, 10)
        self.create_subscription(MarkerArray, '/vision/right_markers', self.right_marker_cb, 10)

        self.bridge = CvBridge()
        
        self.current_frame = None
        self.active_camera = "Front"
        
        self.dock_info = None
        self.left_markers = {}
        self.right_markers = {}
        
        self.latest_log = ""
        self.task_state = "IDLE"

    def front_cam_cb(self, msg):
        if self.active_camera == "Front": self.process_image(msg)
    def left_cam_cb(self, msg):
        if self.active_camera == "Left": self.process_image(msg)
    def right_cam_cb(self, msg):
        if self.active_camera == "Right": self.process_image(msg)

    def process_image(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def dock_pose_cb(self, msg):
        self.dock_info = msg

    def left_marker_cb(self, msg):
        self.left_markers.clear()
        for m in msg.markers: self.left_markers[m.id] = m.pose

    def right_marker_cb(self, msg):
        self.right_markers.clear()
        for m in msg.markers: self.right_markers[m.id] = m.pose

    def send_goal(self, mode, item, pickup, dropoff, feedback_cb, result_cb):
        goal_msg = RunDelivery.Goal()
        goal_msg.task_mode = mode
        goal_msg.item_type = item
        goal_msg.pickup_loc = pickup
        goal_msg.dropoff_loc = dropoff

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, result_cb))

    def goal_response_callback(self, future, result_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.task_state = "REJECTED"
            return
        self.task_state = "ACCEPTED"
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(result_cb)


class HospitalGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.db = HospitalDB()
        self.setWindowTitle("🏥 Smart Hospital Robot Controller (PyQt6)")
        self.resize(1200, 800)
        
        # Style
        self.setStyleSheet("""
            QMainWindow { background-color: #f0f2f5; }
            QLabel { font-size: 14px; color: #333; }
            QGroupBox { font-weight: bold; border: 1px solid #ccc; border-radius: 8px; margin-top: 10px; background-color: white; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background-color: #007bff; color: white; border-radius: 5px; padding: 8px; font-weight: bold; }
            QPushButton:hover { background-color: #0056b3; }
            QPushButton#stopBtn { background-color: #dc3545; }
            QPushButton#stopBtn:hover { background-color: #c82333; }
            QProgressBar { border: 1px solid #bbb; border-radius: 5px; text-align: center; }
            QProgressBar::chunk { background-color: #28a745; width: 20px; }
            QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: Consolas; }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ---------------- Left Panel ----------------
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setFixedWidth(300)

        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        self.battery_bar = QProgressBar()
        self.battery_bar.setValue(100)
        self.battery_bar.setFormat("Battery: %p%")
        status_layout.addWidget(self.battery_bar)
        self.db_stats_label = QLabel("Loading stats...")
        status_layout.addWidget(self.db_stats_label)
        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)

        config_group = QGroupBox("Task Configuration")
        config_layout = QVBoxLayout()
        
        config_layout.addWidget(QLabel("Pickup Location:"))
        self.pickup_combo = QComboBox()
        self.pickup_combo.addItems(self.db.get_all_locations())
        config_layout.addWidget(self.pickup_combo)

        config_layout.addWidget(QLabel("Dropoff Location:"))
        self.dropoff_combo = QComboBox()
        self.dropoff_combo.addItems(self.db.get_all_locations())
        if self.dropoff_combo.count() > 1: self.dropoff_combo.setCurrentIndex(1)
        config_layout.addWidget(self.dropoff_combo)

        config_layout.addWidget(QLabel("Item Type:"))
        self.item_combo = QComboBox()
        self.item_combo.addItems(self.db.get_all_items())
        config_layout.addWidget(self.item_combo)
        config_group.setLayout(config_layout)
        left_layout.addWidget(config_group)

        action_group = QGroupBox("Commands")
        action_layout = QVBoxLayout()
        self.btn_full = QPushButton("🚀 Run Full Sequence")
        self.btn_full.clicked.connect(lambda: self.execute_task("ALL"))
        action_layout.addWidget(self.btn_full)
        
        self.btn_pick = QPushButton("🖐️ Pick Only")
        self.btn_pick.clicked.connect(lambda: self.execute_task("PICK"))
        action_layout.addWidget(self.btn_pick)
        
        self.btn_home = QPushButton("🏠 Go Home")
        self.btn_home.clicked.connect(lambda: self.execute_task("NAV_HOME"))
        action_layout.addWidget(self.btn_home)

        self.btn_stop = QPushButton("🛑 EMERGENCY STOP")
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.clicked.connect(self.emergency_stop)
        action_layout.addWidget(self.btn_stop)
        action_group.setLayout(action_layout)
        left_layout.addWidget(action_group)
        left_layout.addStretch()
        main_layout.addWidget(left_panel)

        # ---------------- Center Panel ----------------
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        
        cam_ctrl_layout = QHBoxLayout()
        self.btn_cam_front = QPushButton("Front View")
        self.btn_cam_left = QPushButton("Left View")
        self.btn_cam_right = QPushButton("Right View")
        self.btn_cam_front.setCheckable(True)
        self.btn_cam_left.setCheckable(True)
        self.btn_cam_right.setCheckable(True)
        self.btn_cam_front.setChecked(True)
        
        self.btn_cam_front.clicked.connect(lambda: self.change_camera("Front"))
        self.btn_cam_left.clicked.connect(lambda: self.change_camera("Left"))
        self.btn_cam_right.clicked.connect(lambda: self.change_camera("Right"))

        cam_ctrl_layout.addWidget(self.btn_cam_front)
        cam_ctrl_layout.addWidget(self.btn_cam_left)
        cam_ctrl_layout.addWidget(self.btn_cam_right)
        center_layout.addLayout(cam_ctrl_layout)

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter) # PyQt6 Enum 변경
        self.video_label.setStyleSheet("background-color: black; border: 2px solid #555;")
        self.video_label.setMinimumSize(640, 480)
        center_layout.addWidget(self.video_label)
        
        self.hud_label = QLabel("Waiting for Camera...")
        self.hud_label.setStyleSheet("color: blue; font-weight: bold;")
        self.hud_label.setAlignment(Qt.AlignmentFlag.AlignCenter) # PyQt6 Enum 변경
        center_layout.addWidget(self.hud_label)
        main_layout.addWidget(center_panel)

        # ---------------- Right Panel ----------------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_panel.setFixedWidth(300)
        log_group = QGroupBox("System Logs")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        right_layout.addWidget(log_group)
        main_layout.addWidget(right_panel)

        # Timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.simulate_battery)
        self.battery_timer.start(5000)
        
        self.is_busy = False
        self.current_task_id = None
        self.update_db_stats()

    def change_camera(self, cam_name):
        self.ros_node.active_camera = cam_name
        self.btn_cam_front.setChecked(cam_name == "Front")
        self.btn_cam_left.setChecked(cam_name == "Left")
        self.btn_cam_right.setChecked(cam_name == "Right")
        self.log_message(f"📷 Switched to {cam_name} Camera")

    def execute_task(self, mode):
        pickup = self.pickup_combo.currentText()
        dropoff = self.dropoff_combo.currentText()
        item = self.item_combo.currentText()
        self.log_message(f"▶️ Command: {mode} | {item}")
        self.is_busy = True
        self.current_task_id = self.db.log_task_start(mode, item, pickup, dropoff)
        self.ros_node.send_goal(mode, item, pickup, dropoff, self.feedback_callback, self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.ros_node.latest_log = feedback_msg.feedback.current_state

    def result_callback(self, future):
        result = future.result().result
        status = "SUCCESS" if result.success else "FAILED"
        self.log_message(f"🏁 Task Finished: {status} - {result.message}")
        self.db.log_task_end(self.current_task_id, status)
        self.update_db_stats()
        self.is_busy = False

    def emergency_stop(self):
        self.log_message("🚨 EMERGENCY STOP TRIGGERED!")

    def log_message(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {msg}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def update_db_stats(self):
        total, success = self.db.get_statistics()
        self.db_stats_label.setText(f"Total Tasks: {total} | Success: {success}")

    def simulate_battery(self):
        current = self.battery_bar.value()
        decrease = 2 if self.is_busy else (1 if current > 90 else 0)
        new_val = max(0, current - decrease)
        self.battery_bar.setValue(new_val)
        if new_val > 50: self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
        elif new_val > 20: self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #ffc107; }")
        else: self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #dc3545; }")

    def update_ui(self):
        if self.ros_node.latest_log:
            self.log_message(f"🤖 Robot: {self.ros_node.latest_log}")
            self.ros_node.latest_log = ""

        frame = self.ros_node.current_frame
        if frame is not None:
            display_frame = frame.copy()
            cam = self.ros_node.active_camera
            h, w, _ = display_frame.shape
            
            if cam == "Front" and self.ros_node.dock_info:
                info = self.ros_node.dock_info
                time_diff = (self.ros_node.get_clock().now() - info.header.stamp).nanoseconds / 1e9
                if time_diff < 1.0:
                    pos = info.pose.position
                    cv2.putText(display_frame, "DOCKING ACTIVE", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                    cv2.putText(display_frame, f"Dist: {pos.z:.2f}m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cx, cy = w // 2, h // 2
                    cv2.line(display_frame, (cx-20, cy), (cx+20, cy), (0, 255, 0), 2)
                    cv2.line(display_frame, (cx, cy-20), (cx, cy+20), (0, 255, 0), 2)

            markers = self.ros_node.left_markers if cam == "Left" else (self.ros_node.right_markers if cam == "Right" else {})
            if markers:
                cv2.putText(display_frame, f"MARKERS: {list(markers.keys())}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2)
                cv2.rectangle(display_frame, (10, 10), (w-10, h-10), (0, 255, 255), 3)

            rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888) # PyQt6 Enum 변경
            
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)) # PyQt6 Enum 변경
            self.hud_label.setText(f"Live Feed: {cam} Camera")
        else:
            self.hud_label.setText("No Signal")

def main():
    rclpy.init()
    ros_node = HospitalControlNode()
    
    app = QApplication(sys.argv)
    gui = HospitalGUI(ros_node)
    
    # QThread로 ROS 스레드 관리 (moveToThread 방식이 더 안전)
    ros_thread = QThread()
    worker = RosWorker(ros_node)
    worker.moveToThread(ros_thread)
    
    # 스레드 시작 시 run 메서드 실행 연결
    ros_thread.started.connect(worker.run)
    worker.finished.connect(ros_thread.quit)
    worker.finished.connect(worker.deleteLater)
    ros_thread.finished.connect(ros_thread.deleteLater)
    
    ros_thread.start()
    
    try:
        sys.exit(app.exec()) # PyQt6에서는 exec() 사용
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()