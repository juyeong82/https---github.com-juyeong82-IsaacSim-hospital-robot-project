import sqlite3
import datetime

class HospitalDB:
    def __init__(self, db_path="hospital.db"):
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.create_tables()
        self.seed_data()

    def create_tables(self):
        cursor = self.conn.cursor()
        
        # 1. 구역/방 정보 (Room DB와 매칭)
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS locations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                zone TEXT NOT NULL,
                type TEXT NOT NULL
            )
        ''')

        # 2. 물품 정보 (Item DB와 매칭)
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS items (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                category TEXT
            )
        ''')

        # 3. 작업 이력 (분석용)
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS tasks (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_mode TEXT,
                item_name TEXT,
                pickup_loc TEXT,
                dropoff_loc TEXT,
                status TEXT,
                start_time TIMESTAMP,
                end_time TIMESTAMP
            )
        ''')
        self.conn.commit()

    def seed_data(self):
        """기본 환경 데이터 주입 (Main Controller와 동일)"""
        cursor = self.conn.cursor()
        
        # [Locations]
        locations = [
            ("Nurse Station A (Base)", "A", "Station"),
            ("Ward 102", "A", "Ward"),
            ("Ward 105", "A", "Ward"),
            ("Main Pharmacy (Central)", "B", "Pharmacy"),
            ("Sub Pharmacy", "B", "Pharmacy"),
            ("Clinical Lab (Zone C)", "C", "Lab")
        ]
        cursor.executemany('INSERT OR IGNORE INTO locations (name, zone, type) VALUES (?, ?, ?)', locations)

        # [Items]
        items = [
            ("Blood Sample", "Biohazard"),
            ("Medicine", "General"),
            ("Narcotics", "Security")
        ]
        cursor.executemany('INSERT OR IGNORE INTO items (name, category) VALUES (?, ?)', items)
        
        self.conn.commit()

    def get_all_locations(self):
        cursor = self.conn.cursor()
        cursor.execute("SELECT name FROM locations")
        return [row[0] for row in cursor.fetchall()]

    def get_all_items(self):
        cursor = self.conn.cursor()
        cursor.execute("SELECT name FROM items")
        return [row[0] for row in cursor.fetchall()]

    def log_task_start(self, mode, item, pickup, dropoff):
        cursor = self.conn.cursor()
        now = datetime.datetime.now()
        cursor.execute('''
            INSERT INTO tasks (task_mode, item_name, pickup_loc, dropoff_loc, status, start_time)
            VALUES (?, ?, ?, ?, 'IN_PROGRESS', ?)
        ''', (mode, item, pickup, dropoff, now))
        self.conn.commit()
        return cursor.lastrowid

    def log_task_end(self, task_id, status):
        cursor = self.conn.cursor()
        now = datetime.datetime.now()
        cursor.execute('''
            UPDATE tasks SET status = ?, end_time = ? WHERE id = ?
        ''', (status, now, task_id))
        self.conn.commit()

    def get_statistics(self):
        """간단한 통계 리턴"""
        cursor = self.conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM tasks WHERE status='SUCCESS'")
        success_count = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM tasks")
        total_count = cursor.fetchone()[0]
        return total_count, success_count