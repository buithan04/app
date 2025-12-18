import sys
import requests
import random
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QLineEdit, QScrollArea, 
                             QGraphicsDropShadowEffect, QProgressBar, QFrame)
from PyQt5.QtCore import (Qt, QTimer, QPropertyAnimation, pyqtProperty, QRect, 
                          QPoint, QThread, pyqtSignal, QEasingCurve)
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen, QPainterPath, QFont

# --- CẤU HÌNH ---
SERVER_URL = "http://127.0.0.1:8000"
COLOR_BG = "#0f172a"      # Nền xanh đen (Slate 900)
COLOR_ACCENT = "#38bdf8"  # Xanh Neon (Sky 400)
COLOR_BUBBLE_USER = "#0284c7"
COLOR_BUBBLE_BOT = "#334155"

# --- WORKER: Giao tiếp Server ngầm ---
class ServerLink(QThread):
    status_signal = pyqtSignal(dict) # Báo pin/trạng thái
    chat_signal = pyqtSignal(dict)   # Báo tin nhắn mới
    
    def __init__(self):
        super().__init__()
        self.chat_queue = None

    def send_chat(self, text):
        self.chat_queue = text

    def run(self):
        while True:
            # 1. Cập nhật trạng thái (Mỗi 1s)
            try:
                resp = requests.get(f"{SERVER_URL}/status", timeout=0.5)
                if resp.status_code == 200:
                    self.status_signal.emit(resp.json())
            except: pass

            # 2. Xử lý tin nhắn (nếu có)
            if self.chat_queue:
                text = self.chat_queue
                self.chat_queue = None # Clear queue
                try:
                    resp = requests.post(f"{SERVER_URL}/chat", json={"text": text}, timeout=35)
                    if resp.status_code == 200:
                        self.chat_signal.emit(resp.json())
                    else:
                        self.chat_signal.emit({"response": "Lỗi Server!", "emotion": "sad"})
                except:
                    self.chat_signal.emit({"response": "Mất kết nối server!", "emotion": "sad"})
            
            self.msleep(1000)

# --- CLASS: MẮT ROBOT NÂNG CAO ---
class AdvancedEye(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(350, 250)
        self._eye_height = 140
        self._pupil_x = 0
        self.emotion = "normal"
        
        # Timer cho hành động ngẫu nhiên (Idle)
        self.idle_timer = QTimer(self)
        self.idle_timer.timeout.connect(self.idle_behavior)
        self.idle_timer.start(2500)

    # Properties cho Animation
    def get_h(self): return self._eye_height
    def set_h(self, val): 
        self._eye_height = val
        self.update()
    
    def get_px(self): return self._pupil_x
    def set_px(self, val):
        self._pupil_x = val
        self.update()

    eye_h = pyqtProperty(int, get_h, set_h)
    pupil_x = pyqtProperty(int, get_px, set_px)

    def set_emotion(self, emo):
        if self.emotion == emo: return
        self.emotion = emo
        
        # Reset con ngươi về giữa
        self.anim_p = QPropertyAnimation(self, b"pupil_x")
        self.anim_p.setDuration(200); self.anim_p.setEndValue(0); self.anim_p.start()

        target = 140
        if emo == "thinking": target = 30    # Nheo mắt
        elif emo == "happy": target = 100    # Cười (vẽ khác)
        elif emo == "sleeping": target = 5   # Nhắm
        elif emo == "surprised": target = 190 # Mở to
        elif emo == "listening": target = 130 # Hơi mở
        
        self.anim = QPropertyAnimation(self, b"eye_h")
        self.anim.setDuration(400)
        self.anim.setEndValue(target)
        self.anim.setEasingCurve(QEasingCurve.OutBack)
        self.anim.start()

    def idle_behavior(self):
        """Tự động nhìn quanh hoặc chớp mắt khi rảnh"""
        if self.emotion not in ["normal", "listening"]: return
        
        action = random.choice(["blink", "look_l", "look_r", "center", "wait", "wait"])
        
        if action == "blink":
            self.anim_b = QPropertyAnimation(self, b"eye_h")
            self.anim_b.setDuration(150)
            self.anim_b.setKeyValueAt(0.5, 5)
            self.anim_b.setEndValue(140)
            self.anim_b.start()
        elif action in ["look_l", "look_r", "center"]:
            target = -40 if action == "look_l" else (40 if action == "look_r" else 0)
            self.anim_p = QPropertyAnimation(self, b"pupil_x")
            self.anim_p.setDuration(500)
            self.anim_p.setEndValue(target)
            self.anim_p.setEasingCurve(QEasingCurve.OutQuad)
            self.anim_p.start()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width()//2, self.height()//2
        spacing = 160
        
        # Chọn màu mắt
        color = QColor(COLOR_ACCENT)
        if self.emotion == "sad": color = QColor("#94a3b8") # Xám
        if self.emotion == "love": color = QColor("#f472b6") # Hồng
        if self.emotion == "angry": color = QColor("#ef4444") # Đỏ

        p.setBrush(color)
        p.setPen(Qt.NoPen)

        # Vẽ hình dáng mắt
        if self.emotion == "love":
            # Vẽ trái tim
            font = QFont("Segoe UI Emoji", 90)
            p.setFont(font); p.setPen(color)
            p.drawText(QRect(cx-spacing-60, cy-60, 120, 120), Qt.AlignCenter, "❤️")
            p.drawText(QRect(cx+spacing-180, cy-60, 120, 120), Qt.AlignCenter, "❤️")
            
        elif self.emotion == "happy":
            # Vẽ mắt cười ^ ^
            path = QPainterPath()
            # Mắt trái
            path.moveTo(cx-spacing-60, cy+20)
            path.quadTo(cx-spacing, cy-80, cx-spacing+60, cy+20)
            # Mắt phải
            path.moveTo(cx+spacing-180, cy+20)
            path.quadTo(cx+spacing-120, cy-80, cx+spacing-60, cy+20)
            
            p.setBrush(Qt.NoBrush)
            pen = QPen(color, 10); pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.drawPath(path)

        else:
            # Vẽ tròng mắt (Sclera)
            rect_l = QRect(0, 0, 130, self._eye_height)
            rect_l.moveCenter(QPoint(cx - spacing//2, cy))
            rect_r = QRect(0, 0, 130, self._eye_height)
            rect_r.moveCenter(QPoint(cx + spacing//2, cy))
            p.drawEllipse(rect_l)
            p.drawEllipse(rect_r)

            # Vẽ con ngươi (Pupil) - Chỉ vẽ khi mắt mở đủ to
            if self._eye_height > 40 and self.emotion != "sleeping":
                p.setBrush(QColor("#0f172a")) # Màu đen
                pupil_size = 40
                
                # Con ngươi trái
                pl = QRect(0,0, pupil_size, pupil_size)
                pl.moveCenter(QPoint(cx - spacing//2 + self._pupil_x, cy))
                p.drawEllipse(pl)
                
                # Con ngươi phải
                pr = QRect(0,0, pupil_size, pupil_size)
                pr.moveCenter(QPoint(cx + spacing//2 + self._pupil_x, cy))
                p.drawEllipse(pr)

# --- GIAO DIỆN CHÍNH ---
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Xavier AI Interface")
        self.resize(500, 850)
        self.setStyleSheet(f"background-color: {COLOR_BG}; color: white; font-family: Segoe UI, Arial;")

        # --- LAYOUT CHÍNH ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        layout.setContentsMargins(0,0,0,0)

        # 1. Header Bar
        header = QWidget()
        header.setFixedHeight(60)
        header.setStyleSheet("background: rgba(0,0,0,0.2);")
        h_layout = QHBoxLayout(header)
        
        self.lbl_status = QLabel("● Waiting...")
        self.lbl_status.setStyleSheet("color: gray; font-weight: bold;")
        
        self.prog_bat = QProgressBar()
        self.prog_bat.setFixedSize(60, 8)
        self.prog_bat.setTextVisible(False)
        self.prog_bat.setStyleSheet(f"QProgressBar{{background:#334155; border-radius:4px;}} QProgressBar::chunk{{background:{COLOR_ACCENT}; border-radius:4px;}}")
        
        h_layout.addWidget(self.lbl_status)
        h_layout.addStretch()
        h_layout.addWidget(QLabel("PWR"))
        h_layout.addWidget(self.prog_bat)
        
        layout.addWidget(header)

        # 2. Eye Area
        self.eye = AdvancedEye()
        layout.addWidget(self.eye, alignment=Qt.AlignCenter)

        # 3. Chat Container (Bo tròn)
        chat_frame = QWidget()
        chat_frame.setStyleSheet("background-color: rgba(30, 41, 59, 0.6); border-top-left-radius: 25px; border-top-right-radius: 25px;")
        v_chat = QVBoxLayout(chat_frame)
        v_chat.setContentsMargins(15, 15, 15, 15)

        # Scroll Area
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setStyleSheet("background: transparent; border: none;")
        self.msg_container = QWidget()
        self.msg_layout = QVBoxLayout(self.msg_container)
        self.msg_layout.addStretch() # Đẩy tin nhắn lên trên
        self.scroll.setWidget(self.msg_container)

        # Input Area
        input_box = QHBoxLayout()
        
        # Nút Thùng rác (Clear Chat)
        btn_clear = QPushButton("🗑️")
        btn_clear.setFixedSize(40, 40)
        btn_clear.setToolTip("Xóa lịch sử & Reset Robot")
        btn_clear.setStyleSheet("background: #475569; border-radius: 20px; font-size: 16px;")
        btn_clear.clicked.connect(self.clear_chat)

        # Ô nhập liệu
        self.txt_input = QLineEdit()
        self.txt_input.setPlaceholderText("Giao tiếp với Xavier...")
        self.txt_input.setStyleSheet("background: #1e293b; color: white; border: 1px solid #475569; border-radius: 20px; padding: 10px 15px;")
        self.txt_input.returnPressed.connect(self.send_msg)
        self.txt_input.textChanged.connect(self.on_typing) # Hiệu ứng khi gõ

        # Nút Gửi
        btn_send = QPushButton("➤")
        btn_send.setFixedSize(45, 45)
        btn_send.setStyleSheet(f"background: {COLOR_ACCENT}; color: {COLOR_BG}; border-radius: 22px; font-weight: bold; font-size: 18px;")
        btn_send.clicked.connect(self.send_msg)

        input_box.addWidget(btn_clear)
        input_box.addWidget(self.txt_input)
        input_box.addWidget(btn_send)

        v_chat.addWidget(self.scroll)
        v_chat.addLayout(input_box)
        layout.addWidget(chat_frame, stretch=1)

        # --- LOGIC THREAD ---
        self.worker = ServerLink()
        self.worker.status_signal.connect(self.update_system_status)
        self.worker.chat_signal.connect(self.receive_ai_msg)
        self.worker.start()

    # --- SỰ KIỆN ---
    def on_typing(self, text):
        """Khi gõ, robot chăm chú lắng nghe"""
        if text and self.eye.emotion != "thinking":
            self.eye.set_emotion("listening")
        elif not text and self.eye.emotion == "listening":
            self.eye.set_emotion("normal")

    def clear_chat(self):
        """Xóa hết tin nhắn và reset trạng thái"""
        # Xóa UI
        while self.msg_layout.count() > 1:
            child = self.msg_layout.takeAt(0)
            if child.widget(): child.widget().deleteLater()
        
        # Reset trạng thái
        self.eye.set_emotion("normal")
        self.add_bubble("Bộ nhớ ngắn hạn đã được xóa. Mình sẵn sàng nghe lệnh mới!", False)

    def send_msg(self):
        text = self.txt_input.text().strip()
        if not text: return
        
        self.add_bubble(text, True)
        self.txt_input.clear()
        
        # Hiệu ứng suy nghĩ
        self.eye.set_emotion("thinking")
        
        # Gửi sang thread xử lý
        self.worker.send_chat(text)

    def receive_ai_msg(self, data):
        response = data.get("response", "...")
        emotion = data.get("emotion", "normal")
        
        self.add_bubble(response, False)
        self.eye.set_emotion(emotion)

    def add_bubble(self, text, is_user):
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        color = COLOR_BUBBLE_USER if is_user else COLOR_BUBBLE_BOT
        align = "margin-left: 40px;" if is_user else "margin-right: 40px;"
        
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {color}; color: white;
                padding: 12px 16px; border-radius: 18px;
                font-size: 14px; line-height: 1.4;
                {align}
            }}
        """)
        # Shadow
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(8); shadow.setColor(QColor(0,0,0,60)); shadow.setOffset(2,2)
        lbl.setGraphicsEffect(shadow)
        
        self.msg_layout.addWidget(lbl)
        # Auto scroll
        QTimer.singleShot(50, lambda: self.scroll.verticalScrollBar().setValue(self.scroll.verticalScrollBar().maximum()))

    def update_system_status(self, data):
        bat = data.get("battery", 0)
        self.prog_bat.setValue(int(bat))
        self.lbl_status.setText(f"● Online | {data.get('temp')}°C")
        self.lbl_status.setStyleSheet(f"color: {COLOR_ACCENT}; font-weight: bold;")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotDashboard()
    window.show()
    sys.exit(app.exec_())
