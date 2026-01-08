#!/usr/bin/env python3
"""
Hospital Robot Database Schema
병원 로봇 시스템의 핵심 데이터베이스 관리 모듈
"""
import sqlite3
from datetime import datetime
from typing import List, Dict, Optional
import json

class HospitalRobotDB:
    def __init__(self, db_path="hospital_robot.db"):
        self.db_path = db_path
        self.init_database()
    
    def get_connection(self):
        """데이터베이스 연결 생성"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row  # Dict 형태로 반환
        return conn
    
    def init_database(self):
        """데이터베이스 초기화 및 테이블 생성"""
        conn = self.get_connection()
        cursor = conn.cursor()
        
        # ============================================
        # 1. 방(Room) 정보 테이블
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS rooms (
                room_id INTEGER PRIMARY KEY AUTOINCREMENT,
                room_name TEXT UNIQUE NOT NULL,
                zone TEXT NOT NULL,  -- A, B, C
                coord_x REAL NOT NULL,
                coord_y REAL NOT NULL,
                coord_z REAL NOT NULL,
                direction TEXT NOT NULL,  -- East, West, North, South
                work_side TEXT NOT NULL,  -- Left, Right
                description TEXT,
                is_active BOOLEAN DEFAULT 1,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # ============================================
        # 2. 물품(Item) 정보 테이블
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS items (
                item_id INTEGER PRIMARY KEY AUTOINCREMENT,
                item_name TEXT UNIQUE NOT NULL,
                marker_id INTEGER UNIQUE NOT NULL,
                offset_x REAL NOT NULL,
                offset_y REAL NOT NULL,
                offset_z REAL NOT NULL,
                category TEXT,  -- Medical, Pharmaceutical, Supply
                description TEXT,
                is_active BOOLEAN DEFAULT 1,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # ============================================
        # 3. 배송 작업(Delivery Task) 이력 테이블
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS delivery_tasks (
                task_id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_mode TEXT NOT NULL,  -- ALL, NAV_PICKUP, etc.
                item_name TEXT NOT NULL,
                pickup_location TEXT NOT NULL,
                dropoff_location TEXT NOT NULL,
                status TEXT NOT NULL,  -- PENDING, IN_PROGRESS, COMPLETED, FAILED, CANCELLED
                current_state TEXT,  -- 현재 진행 단계
                error_message TEXT,
                battery_start REAL,
                battery_end REAL,
                distance_traveled REAL,
                duration_seconds REAL,
                started_at TIMESTAMP,
                completed_at TIMESTAMP,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (item_name) REFERENCES items(item_name),
                FOREIGN KEY (pickup_location) REFERENCES rooms(room_name),
                FOREIGN KEY (dropoff_location) REFERENCES rooms(room_name)
            )
        """)
        
        # ============================================
        # 4. 로봇 상태(Robot Status) 로그 테이블
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS robot_status_log (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id INTEGER,
                battery_level REAL NOT NULL,
                position_x REAL,
                position_y REAL,
                position_z REAL,
                orientation_w REAL,
                current_action TEXT,  -- IDLE, NAVIGATING, DOCKING, PICKING, PLACING
                camera_active TEXT,  -- None, Left, Right, Front
                arm_status TEXT,  -- HOME, PICKING, PLACING, STOWED
                log_message TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (task_id) REFERENCES delivery_tasks(task_id)
            )
        """)
        
        # ============================================
        # 5. 시스템 설정(System Config) 테이블
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS system_config (
                config_key TEXT PRIMARY KEY,
                config_value TEXT NOT NULL,
                description TEXT,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # ============================================
        # 6. 카메라 피드 메타데이터 (선택적)
        # ============================================
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS camera_snapshots (
                snapshot_id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id INTEGER,
                camera_name TEXT NOT NULL,  -- left, right, front
                image_path TEXT,
                detected_markers TEXT,  -- JSON 형태로 저장
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (task_id) REFERENCES delivery_tasks(task_id)
            )
        """)
        
        conn.commit()
        conn.close()
        print("✅ Database initialized successfully!")
    
    # ============================================
    # Room 관련 메서드
    # ============================================
    def insert_room(self, room_name: str, zone: str, coord_x: float, coord_y: float, 
                    coord_z: float, direction: str, work_side: str, description: str = ""):
        """방 정보 삽입"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            INSERT OR REPLACE INTO rooms 
            (room_name, zone, coord_x, coord_y, coord_z, direction, work_side, description)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (room_name, zone, coord_x, coord_y, coord_z, direction, work_side, description))
        conn.commit()
        conn.close()
    
    def get_all_rooms(self) -> List[Dict]:
        """모든 방 정보 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM rooms WHERE is_active = 1 ORDER BY zone, room_name")
        rooms = [dict(row) for row in cursor.fetchall()]
        conn.close()
        return rooms
    
    def get_room_by_name(self, room_name: str) -> Optional[Dict]:
        """특정 방 정보 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM rooms WHERE room_name = ?", (room_name,))
        room = cursor.fetchone()
        conn.close()
        return dict(room) if room else None
    
    # ============================================
    # Item 관련 메서드
    # ============================================
    def insert_item(self, item_name: str, marker_id: int, offset_x: float, 
                    offset_y: float, offset_z: float, category: str = "", description: str = ""):
        """물품 정보 삽입"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            INSERT OR REPLACE INTO items 
            (item_name, marker_id, offset_x, offset_y, offset_z, category, description)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (item_name, marker_id, offset_x, offset_y, offset_z, category, description))
        conn.commit()
        conn.close()
    
    def get_all_items(self) -> List[Dict]:
        """모든 물품 정보 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM items WHERE is_active = 1 ORDER BY category, item_name")
        items = [dict(row) for row in cursor.fetchall()]
        conn.close()
        return items
    
    def get_item_by_name(self, item_name: str) -> Optional[Dict]:
        """특정 물품 정보 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM items WHERE item_name = ?", (item_name,))
        item = cursor.fetchone()
        conn.close()
        return dict(item) if item else None
    
    # ============================================
    # Delivery Task 관련 메서드
    # ============================================
    def create_task(self, task_mode: str, item_name: str, pickup_location: str, 
                    dropoff_location: str, battery_start: float = 100.0) -> int:
        """배송 작업 생성"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO delivery_tasks 
            (task_mode, item_name, pickup_location, dropoff_location, status, battery_start, started_at)
            VALUES (?, ?, ?, ?, 'PENDING', ?, ?)
        """, (task_mode, item_name, pickup_location, dropoff_location, battery_start, datetime.now()))
        task_id = cursor.lastrowid
        conn.commit()
        conn.close()
        return task_id
    
    def update_task_status(self, task_id: int, status: str, current_state: str = "", 
                          error_message: str = "", battery_end: float = None):
        """작업 상태 업데이트"""
        conn = self.get_connection()
        cursor = conn.cursor()
        
        updates = ["status = ?", "current_state = ?"]
        params = [status, current_state]
        
        if error_message:
            updates.append("error_message = ?")
            params.append(error_message)
        
        if battery_end is not None:
            updates.append("battery_end = ?")
            params.append(battery_end)
        
        if status in ["COMPLETED", "FAILED", "CANCELLED"]:
            updates.append("completed_at = ?")
            params.append(datetime.now())
        
        params.append(task_id)
        
        cursor.execute(f"""
            UPDATE delivery_tasks 
            SET {', '.join(updates)}
            WHERE task_id = ?
        """, params)
        conn.commit()
        conn.close()
    
    def get_task(self, task_id: int) -> Optional[Dict]:
        """특정 작업 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM delivery_tasks WHERE task_id = ?", (task_id,))
        task = cursor.fetchone()
        conn.close()
        return dict(task) if task else None
    
    def get_recent_tasks(self, limit: int = 50) -> List[Dict]:
        """최근 작업 이력 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            SELECT * FROM delivery_tasks 
            ORDER BY created_at DESC 
            LIMIT ?
        """, (limit,))
        tasks = [dict(row) for row in cursor.fetchall()]
        conn.close()
        return tasks
    
    def get_task_statistics(self) -> Dict:
        """작업 통계 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        
        # 전체 작업 수
        cursor.execute("SELECT COUNT(*) as total FROM delivery_tasks")
        total = cursor.fetchone()['total']
        
        # 상태별 작업 수
        cursor.execute("""
            SELECT status, COUNT(*) as count 
            FROM delivery_tasks 
            GROUP BY status
        """)
        status_counts = {row['status']: row['count'] for row in cursor.fetchall()}
        
        # 오늘의 작업 수
        cursor.execute("""
            SELECT COUNT(*) as today_count 
            FROM delivery_tasks 
            WHERE DATE(created_at) = DATE('now')
        """)
        today_count = cursor.fetchone()['today_count']
        
        # 평균 배터리 소모
        cursor.execute("""
            SELECT AVG(battery_start - battery_end) as avg_battery_consumption
            FROM delivery_tasks 
            WHERE battery_end IS NOT NULL
        """)
        avg_battery = cursor.fetchone()['avg_battery_consumption'] or 0
        
        # 평균 소요 시간 (초)
        cursor.execute("""
            SELECT AVG(
                (julianday(completed_at) - julianday(started_at)) * 86400
            ) as avg_duration
            FROM delivery_tasks 
            WHERE completed_at IS NOT NULL AND started_at IS NOT NULL
        """)
        avg_duration = cursor.fetchone()['avg_duration'] or 0
        
        conn.close()
        
        return {
            "total_tasks": total,
            "status_counts": status_counts,
            "today_tasks": today_count,
            "avg_battery_consumption": round(avg_battery, 2),
            "avg_duration_seconds": round(avg_duration, 2)
        }
    
    # ============================================
    # Robot Status Log 관련 메서드
    # ============================================
    def log_robot_status(self, task_id: Optional[int], battery_level: float, 
                         position_x: float = None, position_y: float = None, position_z: float = None,
                         current_action: str = "IDLE", camera_active: str = "None", 
                         arm_status: str = "HOME", log_message: str = ""):
        """로봇 상태 로깅"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO robot_status_log 
            (task_id, battery_level, position_x, position_y, position_z, 
             current_action, camera_active, arm_status, log_message)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (task_id, battery_level, position_x, position_y, position_z, 
              current_action, camera_active, arm_status, log_message))
        conn.commit()
        conn.close()
    
    def get_task_logs(self, task_id: int) -> List[Dict]:
        """특정 작업의 로그 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            SELECT * FROM robot_status_log 
            WHERE task_id = ? 
            ORDER BY timestamp ASC
        """, (task_id,))
        logs = [dict(row) for row in cursor.fetchall()]
        conn.close()
        return logs
    
    # ============================================
    # System Config 관련 메서드
    # ============================================
    def set_config(self, key: str, value: str, description: str = ""):
        """시스템 설정 저장"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            INSERT OR REPLACE INTO system_config 
            (config_key, config_value, description, updated_at)
            VALUES (?, ?, ?, ?)
        """, (key, value, description, datetime.now()))
        conn.commit()
        conn.close()
    
    def get_config(self, key: str) -> Optional[str]:
        """시스템 설정 조회"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT config_value FROM system_config WHERE config_key = ?", (key,))
        result = cursor.fetchone()
        conn.close()
        return result['config_value'] if result else None


# ============================================
# 초기 데이터 삽입 함수
# ============================================
def populate_initial_data(db: HospitalRobotDB):
    """초기 데이터 삽입 (방, 물품 정보)"""
    
    # Room Data (main_controller.py의 room_db 기반)
    rooms = [
        ("Nurse Station A (Base)", "A", 23.129, 9.392, 0.0, "East", "Left", "간호사 스테이션 - 로봇 대기소"),
        ("Ward 102", "A", 24.62435, 14.62949, 0.0, "East", "Right", "병실 102호"),
        ("Ward 105", "A", 13.37842, 10.91591, 0.0, "West", "Left", "병실 105호"),
        ("Main Pharmacy (Central)", "B", -9.0, 5.07121, 0.0, "West", "Left", "중앙 약제실"),
        ("Sub Pharmacy", "B", -2.5, 5.07121, 0.0, "West", "Right", "제2 약제실"),
        ("Clinical Lab (Zone C)", "C", -11.61633, 16.26268, 0.0, "North", "Right", "진단검사의학실"),
    ]
    
    for room in rooms:
        db.insert_room(*room)
    
    # Item Data (main_controller.py의 item_db 기반)
    items = [
        ("Blood Sample", 0, 0.0, 0.03, -0.04, "Medical", "혈액 검체 튜브"),
        ("Medicine", 1, 0.0, 0.0, -0.06, "Pharmaceutical", "일반 의약품"),
        ("Narcotics", 2, 0.0, 0.05, -0.02, "Pharmaceutical", "마약류 (보안 필요)"),
    ]
    
    for item in items:
        db.insert_item(*item)
    
    # System Config
    db.set_config("home_x", "0.0", "홈 위치 X 좌표")
    db.set_config("home_y", "0.0", "홈 위치 Y 좌표")
    db.set_config("home_z", "0.0", "홈 위치 Z 좌표")
    db.set_config("battery_drain_per_meter", "0.5", "미터당 배터리 소모율 (%)")
    
    print("✅ Initial data populated successfully!")


# ============================================
# 테스트 코드
# ============================================
if __name__ == "__main__":
    # 데이터베이스 초기화
    db = HospitalRobotDB("hospital_robot.db")
    
    # 초기 데이터 삽입
    populate_initial_data(db)
    
    # 테스트: 방 조회
    print("\n📍 Rooms:")
    for room in db.get_all_rooms():
        print(f"  - {room['room_name']} (Zone {room['zone']})")
    
    # 테스트: 물품 조회
    print("\n📦 Items:")
    for item in db.get_all_items():
        print(f"  - {item['item_name']} (Marker ID: {item['marker_id']})")
    
    # 테스트: 작업 생성
    print("\n🚀 Creating test task...")
    task_id = db.create_task(
        task_mode="ALL",
        item_name="Blood Sample",
        pickup_location="Nurse Station A (Base)",
        dropoff_location="Clinical Lab (Zone C)",
        battery_start=100.0
    )
    print(f"  Created Task ID: {task_id}")
    
    # 테스트: 로그 생성
    db.log_robot_status(
        task_id=task_id,
        battery_level=98.5,
        current_action="NAVIGATING",
        log_message="Moving to pickup location"
    )
    
    # 테스트: 통계 조회
    print("\n📊 Statistics:")
    stats = db.get_task_statistics()
    for key, value in stats.items():
        print(f"  - {key}: {value}")
