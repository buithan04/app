import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import random
import threading
import time
import requests # Cần pip install requests

# --- CẤU HÌNH ---
app = FastAPI(title="Robot Brain AI", version="3.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Cấu hình Ollama (AI Local trên Jetson)
OLLAMA_API = "http://localhost:11434/api/chat"
AI_MODEL = "qwen2.5:7b" # Model bạn đã tải vào NVMe

# Tính cách Robot
SYSTEM_PROMPT = """
Bạn là Xavier, một trợ lý robot thông minh chạy trên nền tảng Nvidia Jetson.
Tính cách: Thân thiện, hơi hài hước, ngắn gọn.
Nhiệm vụ: Trả lời câu hỏi bằng Tiếng Việt (tối đa 2-3 câu).
Lưu ý: Nếu người dùng hỏi về cảm xúc, hãy thể hiện rõ ràng.
"""

# --- DATA MODELS ---
class NavigationTarget(BaseModel):
    location: str

class ChatInput(BaseModel):
    text: str

# --- TRẠNG THÁI ROBOT ---
robot_state = {
    "battery": 100.0,
    "temp": 42.0,
    "mode": "IDLE",
    "current_emotion": "normal",
    "last_response": ""
}

# --- AI & EMOTION LOGIC ---
def analyze_emotion(text):
    """Phân tích cảm xúc dựa trên từ khóa trong câu trả lời"""
    text = text.lower()
    if any(x in text for x in ["haha", "vui", "tuyệt", "cười", "hay"]):
        return "happy"
    if any(x in text for x in ["buồn", "xin lỗi", "tiếc", "khóc", "khổ"]):
        return "sad"
    if any(x in text for x in ["yêu", "thương", "quý", "tim", "love"]):
        return "love"
    if any(x in text for x in ["wow", "bất ngờ", "thật sao", "!", "???"]):
        return "surprised"
    if any(x in text for x in ["ngủ", "mệt", "tạm biệt", "bye"]):
        return "sleeping"
    if any(x in text for x in ["giận", "bực", "cút"]):
        return "angry"
    return "normal"

def query_ollama(text):
    """Gửi tin nhắn sang Ollama"""
    payload = {
        "model": AI_MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": text}
        ],
        "stream": False
    }
    try:
        # Timeout 30s để tránh treo server nếu AI tính quá lâu
        response = requests.post(OLLAMA_API, json=payload, timeout=30)
        if response.status_code == 200:
            return response.json()["message"]["content"]
        return "Hệ thống AI đang bận (Ollama Error)."
    except Exception as e:
        print(f"Lỗi AI: {e}")
        return "Không kết nối được với não bộ AI (Check Ollama)."

# --- THREAD GIẢ LẬP ---
def simulation_loop():
    while True:
        # Giả lập pin tụt
        if robot_state["battery"] > 0:
            robot_state["battery"] = round(robot_state["battery"] - 0.05, 1)
        
        # Giả lập nhiệt độ
        change = random.uniform(-0.5, 0.5)
        robot_state["temp"] = round(max(38.0, min(65.0, robot_state["temp"] + change)), 1)
        time.sleep(1)

threading.Thread(target=simulation_loop, daemon=True).start()

# --- API ENDPOINTS ---

@app.get("/status")
def get_status():
    return robot_state

@app.post("/chat")
def chat_with_robot(data: ChatInput):
    print(f"[USER]: {data.text}")
    
    # 1. Đánh dấu đang suy nghĩ
    robot_state["current_emotion"] = "thinking"
    
    # 2. Gọi AI
    ai_response = query_ollama(data.text)
    
    # 3. Phân tích cảm xúc
    emotion = analyze_emotion(ai_response)
    robot_state["current_emotion"] = emotion
    robot_state["last_response"] = ai_response
    
    print(f"[BOT]: {ai_response} -> Emotion: {emotion}")
    
    return {
        "response": ai_response,
        "emotion": emotion
    }

# Giữ lại các API cũ để tương thích code cũ nếu cần
@app.post("/navigate")
def navigate(target: NavigationTarget):
    robot_state["mode"] = "NAVIGATING"
    return {"status": "accepted", "target": target.location}

@app.post("/set_emotion/{emotion}")
def set_manual_emotion(emotion: str):
    robot_state["current_emotion"] = emotion
    return {"status": "ok"}

if __name__ == "__main__":
    print("🚀 Server AI đang khởi động trên port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)