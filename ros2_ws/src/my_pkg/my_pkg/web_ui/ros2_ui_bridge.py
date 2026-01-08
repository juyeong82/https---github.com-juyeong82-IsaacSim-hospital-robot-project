#!/usr/bin/env python3
"""
ROS2-UI Bridge Server
ROS2 노드와 웹 UI를 연결하는 브릿지 서버
FastAPI + WebSocket으로 실시간 통신 제공
"""
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from typing import List, Dict, Optional
import json
import threading
import time
from datetime import datetime
import base64
import cv2
import numpy as np

# 데이터베이스 및 ROS 메시지 import
from hospital_robot_db import HospitalRobotDB
from moma_interfaces.action import RunDelivery
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# FastAPI 앱 생성
app = FastAPI(title="Hospital Robot Control API")

# CORS 설정 (개발 환경용)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 전역 변수
db = HospitalRobotDB("hospital_robot.db")
active_connections: List[WebSocket] = []
ros_bridge_node = None


# ============================================
# ROS2 브릿지 노드
# ============================================
class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_ui_bridge')
        
        # Action Client
        self.delivery_client = ActionClient(self, RunDelivery, 'run_delivery')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 카메라 구독
        self.latest_images = {
            'front': None,
            'left': None,
            'right': None
        }
        
        self.create_subscription(Image, '/front_camera/rgb', 
                                lambda msg: self.image_callback(msg, 'front'), 10)
        self.create_subscription(Image, '/left_camera/rgb', 
                                lambda msg: self.image_callback(msg, 'left'), 10)
        self.create_subscription(Image, '/right_camera/rgb', 
                                lambda msg: self.image_callback(msg, 'right'), 10)
        
        # 오도메트리 구독 (위치 정보)
        self.latest_odom = None
        self.create_subscription(Odometry, '/chassis/odom', self.odom_callback, 10)
        
        # 배터리 시뮬레이션
        self.current_battery = 100.0
        self.last_position = None
        
        self.current_task_id = None
        self.current_feedback = ""
        
        self.get_logger().info("✅ ROS2 Bridge Node Ready!")
    
    def image_callback(self, msg, camera_name):
        """카메라 이미지 수신"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # JPEG로 인코딩하여 저장 (대역폭 절약)
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.latest_images[camera_name] = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error ({camera_name}): {e}")
    
    def odom_callback(self, msg):
        """오도메트리 수신 및 배터리 계산"""
        current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        # 이전 위치가 있으면 거리 계산
        if self.last_position is not None:
            distance = np.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2
            )
            # 배터리 소모 시뮬레이션 (미터당 0.5% 소모)
            battery_drain = distance * 0.5
            self.current_battery = max(0.0, self.current_battery - battery_drain)
            
            # 데이터베이스에 로그 저장
            if self.current_task_id:
                db.log_robot_status(
                    task_id=self.current_task_id,
                    battery_level=self.current_battery,
                    position_x=current_pos[0],
                    position_y=current_pos[1],
                    position_z=msg.pose.pose.position.z,
                    current_action="NAVIGATING" if distance > 0.01 else "IDLE",
                    log_message=f"Battery: {self.current_battery:.1f}%"
                )
        
        self.last_position = current_pos
        self.latest_odom = msg
    
    async def send_delivery_goal(self, task_mode: str, item_type: str, 
                                  pickup_loc: str, dropoff_loc: str, task_id: int):
        """배송 목표 전송"""
        self.current_task_id = task_id
        
        # 액션 서버가 준비될 때까지 대기
        if not self.delivery_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            db.update_task_status(task_id, "FAILED", error_message="Action server timeout")
            return False
        
        # Goal 생성
        goal_msg = RunDelivery.Goal()
        goal_msg.task_mode = task_mode
        goal_msg.item_type = item_type
        goal_msg.pickup_loc = pickup_loc
        goal_msg.dropoff_loc = dropoff_loc
        
        self.get_logger().info(f"Sending goal: {task_mode} {item_type} from {pickup_loc} to {dropoff_loc}")
        
        # 작업 시작 알림
        db.update_task_status(task_id, "IN_PROGRESS", current_state="Starting...")
        await broadcast_message({
            "type": "task_started",
            "task_id": task_id,
            "battery": self.current_battery
        })
        
        # Goal 전송
        send_goal_future = self.delivery_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: asyncio.create_task(
                self.feedback_callback(feedback_msg, task_id)
            )
        )
        
        # Goal 수락 대기
        goal_handle = await asyncio.wrap_future(send_goal_future)
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            db.update_task_status(task_id, "FAILED", error_message="Goal rejected")
            return False
        
        # 결과 대기
        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        
        # 결과 처리
        if result.result.success:
            db.update_task_status(
                task_id, "COMPLETED", 
                current_state="Completed", 
                battery_end=self.current_battery
            )
            await broadcast_message({
                "type": "task_completed",
                "task_id": task_id,
                "message": result.result.message,
                "battery": self.current_battery
            })
            return True
        else:
            db.update_task_status(
                task_id, "FAILED", 
                error_message=result.result.message,
                battery_end=self.current_battery
            )
            await broadcast_message({
                "type": "task_failed",
                "task_id": task_id,
                "message": result.result.message,
                "battery": self.current_battery
            })
            return False
    
    async def feedback_callback(self, feedback_msg, task_id):
        """피드백 수신 및 브로드캐스트"""
        feedback = feedback_msg.feedback
        current_state = feedback.current_state
        
        self.current_feedback = current_state
        
        # 데이터베이스 업데이트
        db.update_task_status(task_id, "IN_PROGRESS", current_state=current_state)
        
        # UI에 브로드캐스트
        await broadcast_message({
            "type": "task_feedback",
            "task_id": task_id,
            "state": current_state,
            "battery": self.current_battery,
            "timestamp": datetime.now().isoformat()
        })
        
        self.get_logger().info(f"📍 Feedback: {current_state}")


# ============================================
# WebSocket 연결 관리
# ============================================
async def broadcast_message(message: Dict):
    """모든 연결된 클라이언트에게 메시지 전송"""
    disconnected = []
    for connection in active_connections:
        try:
            await connection.send_json(message)
        except:
            disconnected.append(connection)
    
    # 끊어진 연결 제거
    for conn in disconnected:
        active_connections.remove(conn)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 엔드포인트"""
    await websocket.accept()
    active_connections.append(websocket)
    
    try:
        # 초기 연결 메시지
        await websocket.send_json({
            "type": "connected",
            "message": "Connected to Hospital Robot System",
            "timestamp": datetime.now().isoformat()
        })
        
        # 주기적으로 카메라 피드 및 상태 전송
        while True:
            if ros_bridge_node:
                # 카메라 이미지 전송
                camera_data = {}
                for camera_name, image_data in ros_bridge_node.latest_images.items():
                    if image_data:
                        camera_data[camera_name] = image_data
                
                # 로봇 상태 전송
                status_data = {
                    "type": "robot_status",
                    "battery": ros_bridge_node.current_battery,
                    "current_feedback": ros_bridge_node.current_feedback,
                    "cameras": camera_data,
                    "timestamp": datetime.now().isoformat()
                }
                
                # 오도메트리 추가
                if ros_bridge_node.latest_odom:
                    odom = ros_bridge_node.latest_odom
                    status_data["position"] = {
                        "x": odom.pose.pose.position.x,
                        "y": odom.pose.pose.position.y,
                        "z": odom.pose.pose.position.z
                    }
                
                await websocket.send_json(status_data)
            
            await asyncio.sleep(0.5)  # 2Hz 업데이트
            
    except WebSocketDisconnect:
        active_connections.remove(websocket)


# ============================================
# REST API 엔드포인트
# ============================================

@app.get("/api/rooms")
async def get_rooms():
    """방 목록 조회"""
    return {"rooms": db.get_all_rooms()}


@app.get("/api/items")
async def get_items():
    """물품 목록 조회"""
    return {"items": db.get_all_items()}


@app.get("/api/tasks")
async def get_tasks(limit: int = 50):
    """작업 이력 조회"""
    return {"tasks": db.get_recent_tasks(limit)}


@app.get("/api/tasks/{task_id}")
async def get_task(task_id: int):
    """특정 작업 조회"""
    task = db.get_task(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return {"task": task}


@app.get("/api/statistics")
async def get_statistics():
    """통계 조회"""
    return db.get_task_statistics()


@app.post("/api/tasks/create")
async def create_task(request: dict):
    """새 작업 생성 및 실행"""
    task_mode = request.get("task_mode", "ALL")
    item_type = request.get("item_type")
    pickup_loc = request.get("pickup_loc")
    dropoff_loc = request.get("dropoff_loc")
    
    if not all([item_type, pickup_loc, dropoff_loc]):
        raise HTTPException(status_code=400, detail="Missing required fields")
    
    # 배터리 체크
    if not ros_bridge_node or ros_bridge_node.current_battery < 20.0:
        raise HTTPException(status_code=400, detail="Battery too low!")
    
    # 데이터베이스에 작업 생성
    task_id = db.create_task(
        task_mode=task_mode,
        item_name=item_type,
        pickup_location=pickup_loc,
        dropoff_location=dropoff_loc,
        battery_start=ros_bridge_node.current_battery
    )
    
    # ROS2 Goal 전송 (비동기)
    asyncio.create_task(
        ros_bridge_node.send_delivery_goal(
            task_mode, item_type, pickup_loc, dropoff_loc, task_id
        )
    )
    
    return {
        "success": True,
        "task_id": task_id,
        "message": "Task created and started"
    }


@app.post("/api/battery/reset")
async def reset_battery():
    """배터리 리셋 (테스트용)"""
    if ros_bridge_node:
        ros_bridge_node.current_battery = 100.0
        return {"success": True, "battery": 100.0}
    return {"success": False}


@app.get("/api/status")
async def get_status():
    """현재 로봇 상태 조회"""
    if not ros_bridge_node:
        return {"status": "offline"}
    
    status = {
        "status": "online",
        "battery": ros_bridge_node.current_battery,
        "current_feedback": ros_bridge_node.current_feedback,
        "current_task_id": ros_bridge_node.current_task_id
    }
    
    if ros_bridge_node.latest_odom:
        odom = ros_bridge_node.latest_odom
        status["position"] = {
            "x": odom.pose.pose.position.x,
            "y": odom.pose.pose.position.y,
            "z": odom.pose.pose.position.z
        }
    
    return status


# ============================================
# ROS2 스피너 (별도 스레드)
# ============================================
def spin_ros2():
    """ROS2 이벤트 루프 실행"""
    executor = MultiThreadedExecutor()
    executor.add_node(ros_bridge_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_bridge_node.destroy_node()


# ============================================
# 앱 시작 및 종료 이벤트
# ============================================
@app.on_event("startup")
async def startup_event():
    """서버 시작 시 ROS2 노드 초기화"""
    global ros_bridge_node
    
    # ROS2 초기화
    rclpy.init()
    ros_bridge_node = ROS2BridgeNode()
    
    # ROS2 스피너를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=spin_ros2, daemon=True)
    ros_thread.start()
    
    print("🚀 Hospital Robot Bridge Server Started!")
    print("   - HTTP API: http://localhost:8000")
    print("   - WebSocket: ws://localhost:8000/ws")
    print("   - Docs: http://localhost:8000/docs")


@app.on_event("shutdown")
async def shutdown_event():
    """서버 종료 시 정리"""
    if ros_bridge_node:
        rclpy.shutdown()


# ============================================
# 메인 실행
# ============================================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
