#!/bin/bash

# Hospital Robot System Launcher
# 병원 로봇 시스템 통합 실행 스크립트

echo "🏥 Hospital Robot Control System Launcher"
echo "=========================================="
echo ""

# 색상 정의
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 1. 데이터베이스 체크
echo -e "${BLUE}[1/4]${NC} Checking database..."
if [ ! -f "hospital_robot.db" ]; then
    echo -e "${YELLOW}Database not found. Creating...${NC}"
    python3 hospital_robot_db.py
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Database created successfully!${NC}"
    else
        echo -e "${RED}❌ Failed to create database${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✅ Database found${NC}"
fi
echo ""

# 2. 패키지 체크
echo -e "${BLUE}[2/4]${NC} Checking Python packages..."
python3 -c "import fastapi, uvicorn, websockets, cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}Some packages are missing. Installing...${NC}"
    pip install -r requirements.txt --break-system-packages
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Packages installed${NC}"
    else
        echo -e "${RED}❌ Failed to install packages${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✅ All packages installed${NC}"
fi
echo ""

# 3. ROS2 환경 체크
echo -e "${BLUE}[3/4]${NC} Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 not sourced!${NC}"
    echo -e "${YELLOW}Please run: source /opt/ros/humble/setup.bash${NC}"
    exit 1
else
    echo -e "${GREEN}✅ ROS2 $ROS_DISTRO detected${NC}"
fi
echo ""

# 4. 실행 옵션 선택
echo -e "${BLUE}[4/4]${NC} Select launch option:"
echo "  1) Launch Bridge Server only (웹 서버만 실행)"
echo "  2) Launch Everything (전체 시스템 실행 - 권장)"
echo "  3) Test Database (데이터베이스 테스트)"
echo "  4) Exit"
echo ""
read -p "Enter option [1-4]: " option

case $option in
    1)
        echo ""
        echo -e "${GREEN}🚀 Launching Bridge Server...${NC}"
        echo -e "${YELLOW}Note: Make sure main_controller.py is running in another terminal!${NC}"
        echo ""
        python3 ros2_ui_bridge.py
        ;;
    
    2)
        echo ""
        echo -e "${GREEN}🚀 Launching Full System...${NC}"
        echo ""
        
        # tmux 세션 생성
        if command -v tmux &> /dev/null; then
            echo -e "${BLUE}Using tmux for multi-window setup${NC}"
            
            # 새 세션 생성
            tmux new-session -d -s hospital_robot
            
            # 창 1: Main Controller
            tmux send-keys -t hospital_robot:0 "python3 main_controller.py" C-m
            tmux rename-window -t hospital_robot:0 "Main Controller"
            
            # 창 2: Bridge Server
            tmux split-window -h -t hospital_robot:0
            tmux send-keys -t hospital_robot:0.1 "sleep 3 && python3 ros2_ui_bridge.py" C-m
            
            # 창 3: 안내 메시지
            tmux split-window -v -t hospital_robot:0.0
            tmux send-keys -t hospital_robot:0.2 "clear" C-m
            tmux send-keys -t hospital_robot:0.2 "echo '🏥 Hospital Robot System'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo '========================'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo '''" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'Open browser and navigate to:'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'file://$(pwd)/hospital_robot_ui.html'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo '''" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'API Documentation:'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'http://localhost:8000/docs'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo '''" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'Press Ctrl+B then D to detach'" C-m
            tmux send-keys -t hospital_robot:0.2 "echo 'Run \"tmux kill-session -t hospital_robot\" to stop all'" C-m
            
            # 세션에 연결
            tmux attach-session -t hospital_robot
            
        else
            echo -e "${YELLOW}tmux not found. Launching in single terminal...${NC}"
            echo -e "${YELLOW}Recommended: Install tmux for better experience${NC}"
            echo ""
            python3 ros2_ui_bridge.py
        fi
        ;;
    
    3)
        echo ""
        echo -e "${GREEN}🧪 Testing Database...${NC}"
        python3 hospital_robot_db.py
        echo ""
        echo -e "${BLUE}Database contents:${NC}"
        sqlite3 hospital_robot.db << EOF
.mode column
.headers on
SELECT 'ROOMS' as table_name;
SELECT room_name, zone, direction FROM rooms LIMIT 5;
SELECT '';
SELECT 'ITEMS' as table_name;
SELECT item_name, marker_id, category FROM items;
SELECT '';
SELECT 'TASKS' as table_name;
SELECT COUNT(*) as total_tasks FROM delivery_tasks;
EOF
        ;;
    
    4)
        echo "Goodbye!"
        exit 0
        ;;
    
    *)
        echo -e "${RED}Invalid option${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}System launcher finished${NC}"
