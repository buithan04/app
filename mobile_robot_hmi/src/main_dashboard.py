import sys
import random
import requests
import time
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QStackedWidget, 
                             QFrame, QGridLayout, QProgressBar, QLineEdit, QTextEdit,
                             QScrollArea, QSizePolicy, QGraphicsDropShadowEffect)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty, QRect, QTime, QPoint, QSize, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen, QPainterPath, QFont, QLinearGradient, QGradient

# --- CẤU HÌNH KẾT NỐI SERVER ---
SERVER_URL = "http://127.0.0.1:8000" # Địa chỉ của FastAPI Backend
POLL_INTERVAL = 1000                 # Chu kỳ cập nhật trạng thái (ms)

# --- CẤU HÌNH MÀU SẮC & FONT (THEME DARK CYAN - FUTURISTIC) ---
COLOR_BG = "#0f172a"          # Nền chính (Deep Navy)
COLOR_ACCENT = "#00d4ff"      # Màu nhấn (Cyan Neon)
COLOR_TEXT = "#e2e8f0"        # Chữ trắng xám
COLOR_BUBBLE_USER = "#0ea5e9" # Màu bóng chat User (Sky Blue)
COLOR_BUBBLE_BOT = "#334155"  # Màu bóng chat Robot (Slate)
COLOR_WARNING = "#f59e0b"     # Màu cảnh báo (Amber)
COLOR_DANGER = "#ef4444"      # Màu nguy hiểm (Red)
FONT_MAIN = "Segoe UI"      

# --- STYLE SHEET (CSS - Glassmorphism) ---
STYLESHEET = f"""
    QMainWindow {{
        background-color: {COLOR_BG};
    }}
    QLabel {{
        color: {COLOR_TEXT};
        font-family: {FONT_MAIN};
    }}
    /* ScrollBar Ẩn nhưng vẫn cuộn được */
    QScrollBar:vertical {{
        border: none;
        background: rgba(0,0,0,0.1);
        width: 8px;
        margin: 0px;
    }}
    QScrollBar::handle:vertical {{
        background: {COLOR_ACCENT};
        min-height: 20px;
        border-radius: 4px;
    }}
    /* Input Field - Viên thuốc */
    QLineEdit {{
        background-color: rgba(30, 41, 59, 0.8);
        border: 2px solid #334155;
        border-radius: 25px; 
        color: white;
        padding: 10px 20px;
        font-size: 16px;
    }}
    QLineEdit:focus {{
        border: 2px solid {COLOR_ACCENT};
    }}
    /* Buttons */
    QPushButton {{
        background-color: rgba(30, 41, 59, 0.6);
        border: 1px solid {COLOR_ACCENT};
        border-radius: 12px;
        color: {COLOR_ACCENT};
        font-weight: bold;
    }}
    QPushButton:hover {{
        background-color: {COLOR_ACCENT};
        color: {COLOR_BG};
    }}
    QPushButton:pressed {{
        background-color: #0099cc;
    }}
    /* Progress Bar */
    QProgressBar {{
        border: 1px solid {COLOR_ACCENT};
        border-radius: 5px;
        text-align: center;
        background-color: #1e293b;
        color: white;
        font-weight: bold;
    }}
    QProgressBar::chunk {{
        background-color: {COLOR_ACCENT};
        border-radius: 4px;
    }}
"""

# --- WORKER THREAD: GIAO TIẾP SERVER (Tránh đơ giao diện) ---
class ServerWorker(QThread):
    status_updated = pyqtSignal(dict) # Signal gửi data về UI
    connection_status = pyqtSignal(bool) # Signal báo trạng thái kết nối

    def run(self):
        while True:
            try:
                # Gọi API status từ Backend
                response = requests.get(f"{SERVER_URL}/status", timeout=0.5)
                if response.status_code == 200:
                    data = response.json()
                    self.status_updated.emit(data)
                    self.connection_status.emit(True)
                else:
                    self.connection_status.emit(False)
            except Exception:
                # Mất kết nối Server
                self.connection_status.emit(False)
            
            time.sleep(POLL_INTERVAL / 1000.0)

# --- CLASS: MẮT ROBOT (ANIMATION CAO CẤP) ---
class RobotEye(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 200)
        self._eye_height = 100
        self.emotion = "normal" 
        
        # Timer chớp mắt tự nhiên
        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.blink)
        self.blink_timer.start(3500)

    def set_emotion(self, emotion):
        if self.emotion != emotion:
            self.emotion = emotion
            self._eye_height = 100 
            self.update()

    def blink(self):
        # Không chớp mắt với các biểu cảm đặc biệt
        no_blink = ["sleeping", "happy", "sad", "love", "angry", "surprised"]
        if self.emotion in no_blink: return
        
        self.anim = QPropertyAnimation(self, b"eye_height")
        self.anim.setDuration(180)
        self.anim.setStartValue(100)
        self.anim.setEndValue(5)
        self.anim.setKeyValueAt(0.5, 5) 
        self.anim.setKeyValueAt(1.0, 100)
        self.anim.start()

    def get_eye_height(self): return self._eye_height
    def set_eye_height(self, h): 
        self._eye_height = h
        self.update()
    
    eye_height = pyqtProperty(int, get_eye_height, set_eye_height)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center_x, center_y = self.width() // 2, self.height() // 2
        spacing, eye_width = 160, 120
        
        glow_pen = QPen(QColor(COLOR_ACCENT))
        glow_pen.setWidth(6)
        painter.setPen(glow_pen)
        painter.setBrush(QBrush(QColor(COLOR_ACCENT)))

        # --- LOGIC VẼ CÁC BIỂU CẢM ---
        if self.emotion == "happy":
            path = QPainterPath()
            # Mắt cười ^ ^
            path.moveTo(center_x - spacing//2 - 60, center_y + 20)
            path.quadTo(center_x - spacing//2, center_y - 80, center_x - spacing//2 + 60, center_y + 20)
            path.moveTo(center_x + spacing//2 - 60, center_y + 20)
            path.quadTo(center_x + spacing//2, center_y - 80, center_x + spacing//2 + 60, center_y + 20)
            painter.setBrush(Qt.NoBrush)
            painter.drawPath(path)

        elif self.emotion == "thinking":
            rect_l = QRect(0, 0, eye_width, 120)
            rect_l.moveCenter(QPoint(center_x - spacing // 2, center_y))
            rect_r = QRect(0, 0, eye_width, 40) # Mắt nheo
            rect_r.moveCenter(QPoint(center_x + spacing // 2, center_y))
            painter.drawEllipse(rect_l)
            painter.drawEllipse(rect_r)

        elif self.emotion == "sad":
            path = QPainterPath()
            # Mắt buồn (cong xuống)
            path.moveTo(center_x - spacing//2 - 60, center_y - 20)
            path.quadTo(center_x - spacing//2, center_y + 60, center_x - spacing//2 + 60, center_y - 20)
            path.moveTo(center_x + spacing//2 - 60, center_y - 20)
            path.quadTo(center_x + spacing//2, center_y + 60, center_x + spacing//2 + 60, center_y - 20)
            painter.setBrush(Qt.NoBrush)
            painter.drawPath(path)

        elif self.emotion == "love":
            # Sử dụng font Emoji để vẽ trái tim
            font = QFont("Segoe UI Emoji", 80)
            if sys.platform == "linux": font = QFont("Noto Color Emoji", 80)
            painter.setFont(font)
            rect_l = QRect(center_x - spacing//2 - 60, center_y - 60, 120, 120)
            rect_r = QRect(center_x + spacing//2 - 60, center_y - 60, 120, 120)
            painter.setPen(QColor("#ff4d4d"))
            painter.drawText(rect_l, Qt.AlignCenter, "❤️")
            painter.drawText(rect_r, Qt.AlignCenter, "❤️")
            
        elif self.emotion == "speaking": 
            # Hiệu ứng rung nhẹ khi nói
            h = self._eye_height + random.randint(-5, 5)
            w = eye_width + random.randint(-3, 3)
            rect_l = QRect(0, 0, w, h)
            rect_l.moveCenter(QPoint(center_x - spacing // 2, center_y))
            rect_r = QRect(0, 0, w, h)
            rect_r.moveCenter(QPoint(center_x + spacing // 2, center_y))
            painter.drawEllipse(rect_l)
            painter.drawEllipse(rect_r)

        else: # Normal, Surprised, Angry, v.v.
            h = 180 if self.emotion == "surprised" else self._eye_height
            rect_l = QRect(0, 0, eye_width, h)
            rect_l.moveCenter(QPoint(center_x - spacing // 2, center_y))
            rect_r = QRect(0, 0, eye_width, h)
            rect_r.moveCenter(QPoint(center_x + spacing // 2, center_y))
            painter.drawEllipse(rect_l)
            painter.drawEllipse(rect_r)

# --- CLASS: CHAT BUBBLE (TIN NHẮN) ---
class ChatBubble(QWidget):
    def __init__(self, text, is_user=True, parent=None):
        super().__init__(parent)
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(10, 5, 10, 5)
        
        self.lbl_msg = QLabel(text)
        self.lbl_msg.setWordWrap(True)
        self.lbl_msg.setStyleSheet(f"""
            background-color: {COLOR_BUBBLE_USER if is_user else COLOR_BUBBLE_BOT};
            color: white;
            padding: 12px 16px;
            border-radius: 18px;
            font-size: 15px;
            border-bottom-{'right' if is_user else 'left'}-radius: 2px;
        """)
        
        # Đổ bóng nhẹ
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(8)
        shadow.setOffset(2, 2)
        shadow.setColor(QColor(0, 0, 0, 60))
        self.lbl_msg.setGraphicsEffect(shadow)

        if is_user:
            self.layout.addStretch()
            self.layout.addWidget(self.lbl_msg)
        else:
            self.layout.addWidget(self.lbl_msg)
            self.layout.addStretch()

# --- CLASS: MÀN HÌNH TƯƠNG TÁC (CHÍNH) ---
class InteractionScreen(QWidget):
    def __init__(self, switch_to_menu_callback, parent=None):
        super().__init__(parent)
        self.switch_to_menu = switch_to_menu_callback
        
        # Layout chính
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. Top Bar
        top_bar = QWidget()
        top_bar.setFixedHeight(60)
        top_layout = QHBoxLayout(top_bar)
        
        # Status Label
        self.lbl_status = QLabel("● Connecting...")
        self.lbl_status.setStyleSheet("color: gray; font-weight: bold;")
        
        # Battery Indicator
        lbl_bat_icon = QLabel("🔋")
        self.progress_bat = QProgressBar()
        self.progress_bat.setRange(0, 100)
        self.progress_bat.setValue(0)
        self.progress_bat.setFixedSize(60, 15)
        self.progress_bat.setTextVisible(False) 
        
        # Nút Menu
        btn_menu = QPushButton("☰ MENU")
        btn_menu.setFixedSize(100, 40)
        btn_menu.setCursor(Qt.PointingHandCursor)
        btn_menu.clicked.connect(self.switch_to_menu)
        
        top_layout.addWidget(self.lbl_status)
        top_layout.addSpacing(20)
        top_layout.addWidget(lbl_bat_icon)
        top_layout.addWidget(self.progress_bat)
        top_layout.addStretch()
        top_layout.addWidget(btn_menu)
        
        # 2. Khu vực Mắt Robot
        self.eye_area = QWidget()
        eye_layout = QVBoxLayout(self.eye_area)
        self.robot_eye = RobotEye()
        eye_layout.addWidget(self.robot_eye, alignment=Qt.AlignCenter)
        
        # 3. Khu vực Chat
        chat_container = QWidget()
        chat_container.setStyleSheet("background-color: rgba(15, 23, 42, 0.6); border-top-left-radius: 20px; border-top-right-radius: 20px;")
        chat_layout = QVBoxLayout(chat_container)
        chat_layout.setContentsMargins(20, 10, 20, 20)
        
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("background: transparent; border: none;")
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_layout.addStretch()
        self.scroll_area.setWidget(self.scroll_content)
        
        input_bar = QHBoxLayout()
        self.txt_input = QLineEdit()
        self.txt_input.setPlaceholderText("Giao tiếp với Robot...")
        self.txt_input.returnPressed.connect(self.handle_user_input)
        
        btn_send = QPushButton("➤")
        btn_send.setFixedSize(50, 45)
        btn_send.setStyleSheet(f"background-color: {COLOR_ACCENT}; color: {COLOR_BG}; border-radius: 22px; font-size: 20px;")
        btn_send.setCursor(Qt.PointingHandCursor)
        btn_send.clicked.connect(self.handle_user_input)
        
        input_bar.addWidget(self.txt_input)
        input_bar.addWidget(btn_send)
        
        chat_layout.addWidget(self.scroll_area, stretch=1)
        chat_layout.addLayout(input_bar)

        main_layout.addWidget(top_bar)
        main_layout.addWidget(self.eye_area, stretch=4)
        main_layout.addWidget(chat_container, stretch=3)

    def update_connection_ui(self, is_online):
        if is_online:
            self.lbl_status.setText("● Online")
            self.lbl_status.setStyleSheet(f"color: {COLOR_ACCENT}; font-weight: bold;")
        else:
            self.lbl_status.setText("● Offline")
            self.lbl_status.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
            self.progress_bat.setValue(0)

    def update_battery(self, level):
        self.progress_bat.setValue(int(level))
        if level < 20:
            self.progress_bat.setStyleSheet(f"QProgressBar::chunk {{ background-color: {COLOR_DANGER}; }}")
        else:
            self.progress_bat.setStyleSheet(f"QProgressBar::chunk {{ background-color: {COLOR_ACCENT}; }}")

    def add_message(self, text, is_user):
        bubble = ChatBubble(text, is_user)
        self.scroll_layout.addWidget(bubble)
        # Tự động cuộn xuống dưới
        QTimer.singleShot(10, lambda: self.scroll_area.verticalScrollBar().setValue(self.scroll_area.verticalScrollBar().maximum()))

    def handle_user_input(self):
        text = self.txt_input.text().strip()
        if not text: return
        
        self.add_message(text, is_user=True)
        self.txt_input.clear()
        
        # Robot suy nghĩ
        self.robot_eye.set_emotion("thinking")
        # Giả lập trễ xử lý 1s
        QTimer.singleShot(1000, lambda: self.bot_respond(text))

    def bot_respond(self, user_text):
        response = ""
        emotion = "happy"
        
        user_text_lower = user_text.lower()
        if "chào" in user_text_lower:
            response = "Xin chào! Mình đã kết nối với Server Jetson Xavier."
            emotion = "happy"
        elif "thời tiết" in user_text_lower:
            response = "Hôm nay trời đẹp. Nhiệt độ cảm biến ổn định."
            emotion = "normal"
        elif "buồn" in user_text_lower:
            response = "Đừng buồn nhé! Mình ở đây với bạn."
            emotion = "love"
        elif "menu" in user_text_lower:
            response = "Đang mở menu chức năng..."
            emotion = "normal"
            QTimer.singleShot(1500, self.switch_to_menu)
        else:
            response = f"Mình đã nhận được lệnh: '{user_text}'"
            emotion = "surprised"

        self.add_message(response, is_user=False)
        self.simulate_speaking(response, emotion)

        # Gửi biểu cảm lên Server (nếu kết nối được)
        try:
            threading.Thread(target=requests.post, args=(f"{SERVER_URL}/set_emotion/{emotion}",)).start()
        except: pass

    def simulate_speaking(self, text, final_emotion):
        self.robot_eye.set_emotion("speaking")
        # Thời gian nói phụ thuộc độ dài câu
        duration = min(len(text) * 80, 4000)
        QTimer.singleShot(duration, lambda: self.robot_eye.set_emotion(final_emotion))

# --- CLASS: DASHBOARD SCREEN (MENU CHỨC NĂNG) ---
class DashboardScreen(QWidget):
    def __init__(self, back_callback, nav_callbacks, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 40, 40, 40)
        
        header = QHBoxLayout()
        btn_back = QPushButton("⬅ Quay lại")
        btn_back.setFixedSize(150, 50)
        btn_back.clicked.connect(back_callback)
        lbl_title = QLabel("ĐIỀU KHIỂN ROBOT")
        lbl_title.setStyleSheet(f"font-size: 28px; font-weight: bold; color: {COLOR_ACCENT};")
        header.addWidget(btn_back)
        header.addStretch()
        header.addWidget(lbl_title)
        
        layout.addLayout(header)
        layout.addSpacing(40)
        
        grid = QGridLayout()
        grid.setSpacing(30)
        
        def create_big_btn(icon, text, callback):
            btn = QPushButton(f"{icon}\n{text}")
            btn.setFixedSize(220, 160)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: rgba(30, 41, 59, 0.5);
                    border: 2px solid #334155;
                    border-radius: 20px;
                    color: white;
                    font-size: 20px;
                }}
                QPushButton:hover {{
                    border-color: {COLOR_ACCENT};
                    color: {COLOR_ACCENT};
                    background-color: rgba(0, 212, 255, 0.1);
                }}
            """)
            btn.clicked.connect(callback)
            return btn

        btn_map = create_big_btn("🗺️", "Dẫn đường", nav_callbacks['map'])
        btn_weather = create_big_btn("☁️", "Thời tiết", nav_callbacks['weather'])
        btn_info = create_big_btn("ℹ️", "Thông tin", nav_callbacks['info'])
        btn_chat = create_big_btn("💬", "Trò chuyện", back_callback) # Nút mới để quay lại chat
        
        # Sắp xếp lưới 2x2
        grid.addWidget(btn_map, 0, 0)
        grid.addWidget(btn_weather, 0, 1)
        grid.addWidget(btn_info, 1, 0)
        grid.addWidget(btn_chat, 1, 1)
        
        container = QWidget()
        container.setLayout(grid)
        layout.addWidget(container, alignment=Qt.AlignCenter)
        layout.addStretch()

# --- SUB SCREENS (CÁC MÀN HÌNH CON) ---
class BaseSubScreen(QWidget):
    def __init__(self, title, back_callback):
        super().__init__()
        self.layout = QVBoxLayout(self)
        btn = QPushButton("⬅ Quay lại")
        btn.setFixedSize(100, 40)
        btn.clicked.connect(back_callback)
        
        header = QHBoxLayout()
        header.addWidget(btn)
        header.addStretch()
        
        self.layout.addLayout(header)
        self.layout.addWidget(QLabel(title, styleSheet="font-size: 24px; font-weight: bold; color: white;"), alignment=Qt.AlignCenter)

class MapScreen(BaseSubScreen):
    def __init__(self, back_callback):
        super().__init__("BẢN ĐỒ & DẪN ĐƯỜNG", back_callback)
        # Placeholder cho ROS Map
        map_area = QLabel("ROS 2 MAP VISUALIZATION\n(Kết nối Rviz2 tại đây)", styleSheet="color: gray; font-size: 18px; border: 2px dashed gray;")
        map_area.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(map_area)

# --- MAIN APP CONTROLLER ---
class MainDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Companion - Jetson Xavier")
        self.resize(1024, 600)
        self.setStyleSheet(STYLESHEET)

        # 1. Khởi tạo Worker Thread (Kết nối Server)
        self.server_worker = ServerWorker()
        self.server_worker.status_updated.connect(self.on_server_data)
        self.server_worker.connection_status.connect(self.on_connection_change)
        self.server_worker.start()

        # 2. Setup UI Stacks
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.screen_interact = InteractionScreen(lambda: self.switch_view(1))
        self.stack.addWidget(self.screen_interact)

        self.screen_menu = DashboardScreen(
            back_callback=lambda: self.switch_view(0),
            nav_callbacks={
                'map': lambda: self.switch_view(2),
                'weather': lambda: print("[CMD] Xem thời tiết"),
                'info': lambda: print("[CMD] Xem thông tin")
            }
        )
        self.stack.addWidget(self.screen_menu)
        self.stack.addWidget(MapScreen(lambda: self.switch_view(1)))

    def switch_view(self, index):
        self.stack.setCurrentIndex(index)
        if index == 0:
            self.screen_interact.robot_eye.set_emotion("normal")

    def on_server_data(self, data):
        # Dữ liệu nhận từ Server: {'battery': 99, 'temp': 45.5, ...}
        bat = data.get("battery", 0)
        self.screen_interact.update_battery(bat)
        
        # Nếu pin yếu < 20%, robot tự động buồn
        if bat < 20:
            self.screen_interact.robot_eye.set_emotion("sad")

    def on_connection_change(self, is_online):
        self.screen_interact.update_connection_ui(is_online)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainDashboard()
    window.show()
    sys.exit(app.exec_())
