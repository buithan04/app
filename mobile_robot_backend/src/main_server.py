import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import random
import threading
import time

# --- CẤU HÌNH APP ---
app = FastAPI(
    title="Robot Control Center (Professional)",
    description="Backend điều khiển Robot Jetson Xavier",
    version="2.0"
)

# Cấu hình CORS (Cho phép mọi nguồn truy cập - Quan trọng khi dev)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- DATA MODELS ---
class NavigationTarget(BaseModel):
    location: str
    coordinate_x: float = 0.0
    coordinate_y: float = 0.0

# --- TRẠNG THÁI ROBOT (GLOBAL STATE) ---
robot_state = {
    "battery": 100.0,      # Pin bắt đầu 100%
    "temp": 42.0,          # Nhiệt độ C
    "wifi_strength": "Strong",
    "mode": "IDLE",        # IDLE, NAVIGATING, ERROR
    "current_emotion": "normal"
}

# --- THREAD MÔ PHỎNG (Background Simulation) ---
def simulation_loop():
    """Mô phỏng pin tụt và nhiệt độ thay đổi theo thời gian"""
    while True:
        # 1. Mô phỏng Pin tụt (0.5% mỗi giây)
        if robot_state["battery"] > 0:
            robot_state["battery"] -= 0.1
            robot_state["battery"] = round(robot_state["battery"], 1)
        
        # 2. Mô phỏng Nhiệt độ dao động (40 - 65 độ C)
        change = random.uniform(-1.0, 1.0)
        new_temp = robot_state["temp"] + change
        robot_state["temp"] = round(max(40.0, min(65.0, new_temp)), 1)
        
        time.sleep(1)

# Chạy luồng mô phỏng ngay khi server khởi động
sim_thread = threading.Thread(target=simulation_loop, daemon=True)
sim_thread.start()

# --- API ENDPOINTS ---

@app.get("/")
def read_root():
    return {"status": "Running", "system": "Jetson Xavier NX"}

@app.get("/status")
def get_robot_status():
    """API trả về toàn bộ trạng thái robot (được gọi liên tục bởi GUI)"""
    return robot_state

@app.post("/set_emotion/{emotion}")
def set_emotion(emotion: str):
    """API nhận lệnh đổi biểu cảm từ GUI"""
    valid_emotions = ["normal", "happy", "sad", "angry", "thinking", "sleeping", "love", "surprised"]
    
    if emotion not in valid_emotions:
        # Nếu gửi biểu cảm lạ, vẫn chấp nhận nhưng log cảnh báo
        print(f"[WARNING] Nhận biểu cảm lạ: {emotion}")
    
    robot_state["current_emotion"] = emotion
    print(f"[GUI CMD] Đổi biểu cảm -> {emotion}")
    return {"status": "success", "emotion": emotion}

@app.post("/navigate")
def navigate_to(target: NavigationTarget):
    """API nhận lệnh di chuyển"""
    print(f"[NAV] Đang di chuyển tới: {target.location}")
    robot_state["mode"] = "NAVIGATING"
    return {"status": "accepted", "target": target.location}

@app.post("/stop")
def emergency_stop(): 
    print("!!! [EMERGENCY] DỪNG KHẨN CẤP ĐƯỢC KÍCH HOẠT !!!")
    robot_state["mode"] = "STOPPED"
    robot_state["current_emotion"] = "surprised" # Robot ngạc nhiên khi bị dừng
    return {"status": "stopped"}

# --- ENTRY POINT ---
if __name__ == "__main__":
    # Chạy server tại 0.0.0.0 để các máy khác trong mạng LAN có thể truy cập
    print("🚀 Server Backend đang khởi động trên port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
