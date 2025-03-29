import threading
import time
import websocket
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import json
import random
import requests
import pygame

from kivy.app import App
from kivy.uix.screenmanager import Screen, ScreenManager, FadeTransition, CardTransition
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivy.uix.widget import Widget
from kivy.graphics import Ellipse, Color, Rectangle, Line, RoundedRectangle
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.uix.camera import Camera
from kivy.graphics.texture import Texture
from kivy.uix.behaviors import ButtonBehavior
from kivy.utils import get_color_from_hex
from kivy.animation import Animation
from websocket import create_connection
from kivy.uix.relativelayout import RelativeLayout
from kivy.properties import NumericProperty

# Set app theme colors
THEME_COLORS = {
    'primary': get_color_from_hex('#03A9F4'),
    'secondary': get_color_from_hex('#2196F3'),
    'accent': get_color_from_hex('#FFAC63'),
    'background': get_color_from_hex('#FFFFFF'),
    'text': get_color_from_hex('#212121'),
    'success': get_color_from_hex('#4CAF50'),
    'error': get_color_from_hex('#F44336'),
    'disabled': get_color_from_hex('#BDBDBD'),
    'highlight': get_color_from_hex('#39FF14')
}

# Set default window size
Window.size = (1920, 1080)
Window.clearcolor = THEME_COLORS['background']

# Global Server Configuration
SERVER_IP = "10.192.31.229"  # BU Guest
# SERVER_IP = "128.197.53.43" # Ethernet
# SERVER_IP = 192.168.1.5 # Netgear

SERVER_PORT = 3000
WS_SERVER_URL = f"ws://{SERVER_IP}:{SERVER_PORT}"
HTTP_SERVER_URL = f"http://{SERVER_IP}:{SERVER_PORT}"

# Initialize pygame mixer once
pygame.mixer.init()

# ------------------ Custom Widgets ------------------
class RoundedButton(ButtonBehavior, BoxLayout):
    def __init__(self, text="", icon=None, bg_color=THEME_COLORS['primary'], text_color=(1, 1, 1, 1),
                 font_size=50, radius=10, size_hint=(1, 1), height=60, **kwargs):
        # Simplify rounded corners: default radius now 10
        super().__init__(size_hint=size_hint, height=height, **kwargs)
        self.bg_color = bg_color
        self.text = text
        self.radius = radius
        # Use simpler color transition
        self.press_color = (self.bg_color[0] * 0.9, self.bg_color[1] * 0.9,
                            self.bg_color[2] * 0.9, self.bg_color[3])

        # Create content layout (static, not recreated on every press)
        content = BoxLayout(orientation='horizontal', padding=10, spacing=10)
        if icon:
            icon_img = Image(source=icon, size_hint=(None, None), size=(30, 30))
            content.add_widget(icon_img)
        label = Label(text=text, font_size=font_size, color=text_color,
                      halign='center', valign='middle', size_hint=(1, 1))
        content.add_widget(label)
        self.add_widget(content)

        # Pre-create canvas instructions for background once
        with self.canvas.before:
            self.button_color = Color(*bg_color)
            self.bg_rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[self.radius])
        self.bind(pos=self.update_rect, size=self.update_rect)

    def update_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size

    def on_press(self):
        # Update existing color instruction using press_color
        self.button_color.rgba = self.press_color

    def on_release(self):
        self.button_color.rgba = self.bg_color

class HeaderBar(BoxLayout):
    def __init__(self, title="SmoothOperator", icon=None, switch_screen_callback=lambda: None, **kwargs):
        kwargs.pop('switch_screen_callback', None)
        super().__init__(orientation='horizontal', size_hint=(1, None), height=60, **kwargs)
        self.switch_screen_callback = switch_screen_callback
        with self.canvas.before:
            self.header_color = Color(*THEME_COLORS['primary'])
            self.rect = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_rect, size=self.update_rect)

        # Create header content
        content = BoxLayout(orientation='horizontal', padding=10, spacing=10)
        if icon:
            icon_img = Image(source=icon, size_hint=(None, None), size=(40, 40))
            content.add_widget(icon_img)
        title_label = Label(text=title, font_size=50, color=(1, 1, 1, 1),
                            halign='left', valign='middle', size_hint=(1, 1))
        content.add_widget(title_label)
        back_button = Button(text="Back", size_hint=(None, None), size=(80, 40),
                             background_color=THEME_COLORS['secondary'], font_size=20)
        back_button.bind(on_press=self.go_back)
        content.add_widget(back_button)
        self.add_widget(content)

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size

    def go_back(self, instance):
        app = App.get_running_app()
        if app.sm.current == "menu":
            app.sm.transition = CardTransition(mode='pop')
            app.sm.current = "face"
        elif app.sm.current != "face":
            app.sm.transition = CardTransition(mode='pop')
            app.sm.current = "menu"

class MouthWidget(Widget):
    mouth_open = NumericProperty(0.1)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas:
            Color(*THEME_COLORS['primary'])
            # Animated line
            self.smile_line = Line(bezier=[], width=4.0)
            # Static line that will not animate
            self.smile_line1 = Line(bezier=[], width=4.0)
        # Initialize the mouth (which sets both lines)
        self.bind(pos=self.update_mouth, size=self.update_mouth, mouth_open=self.update_mouth)
        self.update_mouth()

    def update_mouth(self, *args):
        width, height = self.width, self.height
        bottom_y = self.y
        smile_width = 0.9
        left_x = self.x + width * (1 - smile_width) / 2
        right_x = self.x + width * (1 + smile_width) / 2
        smile_height = height * self.mouth_open * 10
        points = [
            left_x, bottom_y,
            left_x + width/4, bottom_y - smile_height,
            right_x - width/4, bottom_y - smile_height,
            right_x, bottom_y
        ]
        # Update the animated line
        self.smile_line.bezier = points

        # Set the static line only once using the computed points
        if not hasattr(self, '_static_line_set'):
            self.smile_line1.bezier = points
            self._static_line_set = True

class MouthWidget1(Widget):
    mouth_open = NumericProperty(0.1)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas:
            Color(*THEME_COLORS['primary'])
            self.smile_line = Line(bezier=[], width=4.0)
        # Compute the mouth shape once—no bindings for continuous updates.
        self.update_mouth()

    def update_mouth(self, *args):
        width, height = self.width, self.height
        bottom_y = self.y
        smile_width = 0.9
        left_x = self.x + width * (1 - smile_width) / 2
        right_x = self.x + width * (1 + smile_width) / 2
        smile_height = height * self.mouth_open * 10
        points = [
            left_x, bottom_y,
            left_x + width/4, bottom_y - smile_height,
            right_x - width/4, bottom_y - smile_height,
            right_x, bottom_y
        ]
        self.smile_line.bezier = points



# ------------------ FaceScreen ------------------
class FaceScreen(Widget):
    def __init__(self, switch_screen_callback, **kwargs):
        super().__init__(**kwargs)
        self.switch_screen_callback = switch_screen_callback
        self.blinking = False
        self.eye_size = (Window.width * 0.25, Window.height * 0.5)
        self.eye_spacing = Window.width * 0.5
        self.eye_offset_x = 0
        self.eye_offset_y = 0

        # Pre-render background once
        with self.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # Create eyes once
        with self.canvas:
            Color(*THEME_COLORS['primary'])
            self.left_eye = Ellipse(size=self.eye_size)
            self.right_eye = Ellipse(size=self.eye_size)
        self.update_positions()
        self.bind(pos=self.update_positions, size=self.on_resize)
        # Reduced blink frequency: every 5 seconds instead of 3
        Clock.schedule_interval(self.blink, 5)

        # Help button (pre-rendered)
        self.help_button = RoundedButton(text="?", bg_color=THEME_COLORS['accent'],
                                           font_size=30, radius=10, size_hint=(None, None), size=(80, 80))
        self.help_button.bind(on_press=lambda x: App.get_running_app().switch_to_help())
        self.add_widget(self.help_button)
        self.bind(pos=self.update_help_button_position, size=self.update_help_button_position)

        # Mouth widget
        self.mouth_widget = MouthWidget(size_hint=(None, None), size=(self.eye_size[0] * 0.8, 30))
        self.mouth_widget1 = MouthWidget1(size_hint=(None, None), size=(self.eye_size[0] * 0.8, 30))

        self.add_widget(self.mouth_widget)
        self.add_widget(self.mouth_widget1)
        self.bind(pos=self.update_mouth_position, size=self.update_mouth_position)

        # # Schedule random audio announcements
        # self.schedule_random_audio()

        # Start remote WebSocket for keyboard commands
        self.start_remote_ws()

    def update_help_button_position(self, *args):
        self.help_button.pos = (self.width - self.help_button.width - 10, 10)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_positions(self, *args):
        left_x = self.center_x - self.eye_spacing / 2 - self.eye_size[0] / 2 + self.eye_offset_x
        right_x = self.center_x + self.eye_spacing / 2 - self.eye_size[0] / 2 + self.eye_offset_x
        left_y = self.center_y - self.eye_size[1] / 2 + self.eye_offset_y
        right_y = self.center_y - self.eye_size[1] / 2 + self.eye_offset_y
        self.left_eye.pos = (left_x, left_y)
        self.right_eye.pos = (right_x, right_y)

    def update_mouth_position(self, *args):
        self.mouth_widget.pos = (self.center_x - self.mouth_widget.width / 2 + self.eye_offset_x,
                                 self.center_y - self.eye_size[1] / 2 + self.eye_offset_y - 20)

    def on_resize(self, *args):
        self.eye_size = (Window.width * 0.25, Window.height * 0.5)
        self.eye_spacing = Window.width * 0.5
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.update_positions()

    def blink(self, dt):
        if not self.blinking:
            self.left_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.right_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.blinking = True
            Clock.schedule_once(self.unblink, 0.2)

    def unblink(self, dt):
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.blinking = False

    # def schedule_random_audio(self, dt=0):
    #     delay = random.randint(10, 30)
    #     Clock.schedule_once(self.random_audio, delay)

    # def random_audio(self, dt):
    #     print("Playing audio NOW")
    #     audio_files = [
    #         "audio/Hi_Im_SmoothOperator.mp3",
    #         "audio/BEEPBEEP.mp3"
    #     ]
    #     chosen_audio = random.choice(audio_files)
    #     self.play_audio(chosen_audio)
    #     self.schedule_random_audio()

    # def play_audio(self, file_path):
    #     print("Playing audio with pygame...")
    #     try:
    #         if pygame.mixer.music.get_busy():
    #             pygame.mixer.music.stop()
    #         pygame.mixer.music.load(file_path)
    #         pygame.mixer.music.play()
    #         self.start_mouth_animation()
    #         Clock.schedule_interval(self.check_audio_finished, 0.1)
    #     except Exception as e:
    #         print("Unable to play audio file:", file_path, "error:", e)

    # def check_audio_finished(self, dt):
    #     if not pygame.mixer.music.get_busy():
    #         self.stop_mouth_animation()
    #         return False
    #     return True
    def check_audio_finished(self, dt):
        if not pygame.mixer.music.get_busy():
            self.stop_mouth_animation()
            if hasattr(self, '_mouth_check_event') and self._mouth_check_event is not None:
                self._mouth_check_event.cancel()
                self._mouth_check_event = None
            return False  # This cancels the scheduled check.
        return True  # Continue checking.


    def start_mouth_animation(self):
        # Reduced animation complexity: slower, less extreme
        self.mouth_anim = Animation(mouth_open=0.6, duration=0.3) + Animation(mouth_open=0.1, duration=0.3)
        self.mouth_anim.repeat = True
        self.mouth_anim.start(self.mouth_widget)

    def stop_mouth_animation(self):
        if hasattr(self, 'mouth_anim'):
            self.mouth_anim.cancel(self.mouth_widget)
        self.mouth_widget.mouth_open = 0.1

    def on_touch_down(self, touch):
        if self.help_button.collide_point(*touch.pos):
            return super().on_touch_down(touch)
        self.switch_screen_callback()
        return super().on_touch_down(touch)

    def start_remote_ws(self):
        WS_SERVER = WS_SERVER_URL
        self.remote_ws = websocket.WebSocketApp(
            WS_SERVER,
            on_message=self.on_remote_message,
            on_error=self.on_remote_error,
            on_close=self.on_remote_close,
            on_open=self.on_remote_open,
            keep_running=True
        )
        threading.Thread(target=self.remote_ws.run_forever, daemon=True).start()

    def on_remote_message(self, ws, message):
        print("FaceScreen remote WS message:", message)
        command = message.strip()
        if command in ['w', 'a', 's', 'd', 'x']:
            Clock.schedule_once(lambda dt: self.process_remote_command(command), 0)
        else:
            print("Received unknown remote command:", command)

    def on_remote_error(self, ws, error):
        print("FaceScreen remote WS error:", error)

    def on_remote_close(self, ws):
        print("FaceScreen remote WS closed")

    def on_remote_open(self, ws):
        print("FaceScreen remote WS opened")

    def process_remote_command(self, command):
        sound_map = {
            'w': 'Watchout',
            's': 'Watchout',
            'a': 'Watchout',
            'd': 'Watchout',
            'x': 'Watchout'
        }
        if command in sound_map:
            sound_manager.play_sound(sound_map[command], face_widget=self)
        
        movement_x = 200
        movement_y = 200  # Enable vertical movement
        
        if command == 'w':
            self.eye_offset_y = movement_y
        elif command == 's':
            self.eye_offset_y = -movement_y
        elif command == 'a':
            self.eye_offset_x = -movement_x
        elif command == 'd':
            self.eye_offset_x = movement_x
        elif command == 'x':
            self.eye_offset_x = 0
            self.eye_offset_y = 0

        self.update_positions()
        self.update_mouth_position()


    def reset_remote_offsets(self):
        self.eye_offset_x = 0
        self.eye_offset_y = 0
        self.update_positions()
        self.update_mouth_position()

# ------------------ ConnectScreen ------------------
class ConnectScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.main_layout = FloatLayout()
        header = HeaderBar(switch_screen_callback=lambda: None, title="Connect to App")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        self.passcode = "{:04d}".format(random.randint(0, 9999))
        instructions = (
            "[b]Connect to App[/b]\n\n"
            "Type the following 4-digit passcode in the app to connect to your Smooth Operator machine:\n\n"
            f"[b]{self.passcode}[/b]\n\n"
            "If you need assistance, please contact support."
        )
        instructions_label = Label(text=instructions, markup=True, font_size=40,
                                   color=THEME_COLORS['text'], halign='center', valign='middle',
                                   pos_hint={'center_x': 0.5, 'center_y': 0.5}, size_hint=(0.8, 0.6))
        instructions_label.bind(size=instructions_label.setter('text_size'))
        self.main_layout.add_widget(instructions_label)
        self.add_widget(self.main_layout)
        self.send_passcode_to_server()
        self.start_ws()

    def send_passcode_to_server(self):
        def post_passcode():
            try:
                url = f"{HTTP_SERVER_URL}/api/connect"
                payload = {"passcode": self.passcode}
                headers = {"Content-Type": "application/json"}
                response = requests.post(url, json=payload, headers=headers, timeout=5)
                print("Sent passcode to server:", response.text)
            except Exception as e:
                print("Error sending passcode:", e)
        threading.Thread(target=post_passcode, daemon=True).start()

    def start_ws(self):
        WS_SERVER = WS_SERVER_URL
        self.ws = websocket.WebSocketApp(
            WS_SERVER,
            on_message=self.on_ws_message,
            on_error=self.on_ws_error,
            on_close=self.on_ws_close,
            on_open=self.on_ws_open,
            keep_running=True
        )
        threading.Thread(target=self.ws.run_forever, daemon=True).start()

    def on_ws_message(self, ws, message):
        print("ConnectScreen received WebSocket message:", message)
        if message.strip() == "AUTH_SUCCESS":
            Clock.schedule_once(lambda dt: self.on_auth_success(), 0)

    def on_ws_error(self, ws, error):
        print("ConnectScreen WebSocket error:", error)

    def on_ws_close(self, ws):
        print("ConnectScreen WebSocket closed")

    def on_ws_open(self, ws):
        print("ConnectScreen WebSocket opened")

    def on_auth_success(self):
        self.main_layout.clear_widgets()
        header = HeaderBar(switch_screen_callback=lambda: None, title="Connection Success!")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        success_label = Label(text="Connection Success Message!", font_size=40,
                              color=THEME_COLORS['success'], pos_hint={'center_x': 0.5, 'center_y': 0.6})
        self.main_layout.add_widget(success_label)
        return_button = RoundedButton(text="Return to SmoothOperator Face", size_hint=(0.5, 0.2),
                                      pos_hint={'center_x': 0.5, 'center_y': 0.3})
        return_button.bind(on_release=self.return_to_face)
        self.main_layout.add_widget(return_button)

    def return_to_face(self, instance):
        app = App.get_running_app()
        app.sm.transition = CardTransition(mode='pop')
        app.sm.current = "face"

# ------------------ MenuScreen ------------------
class MenuScreen(FloatLayout):
    def __init__(self, switch_to_manual, switch_to_qr, switch_to_help, switch_to_connect, switch_to_load_luggage, **kwargs):
        super().__init__(**kwargs)
        self.switch_to_manual = switch_to_manual
        self.switch_to_qr = switch_to_qr
        self.switch_to_help = switch_to_help
        self.switch_to_connect = switch_to_connect
        self.switch_to_load_luggage = switch_to_load_luggage
        with self.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)
        self.build_ui()

    def build_ui(self):
        self.clear_widgets()
        header = HeaderBar(switch_screen_callback=lambda: None, title="Menu")
        header.pos_hint = {'top': 1}
        self.add_widget(header)
        button_layout = BoxLayout(orientation='vertical', spacing=20, padding=[50, 50],
                                  size_hint=(0.8, 0.7), pos_hint={'center_x': 0.5, 'center_y': 0.45})
        load_luggage_btn = RoundedButton(text="Load Luggage", bg_color=THEME_COLORS['success'],
                                          font_size=50, height=80)
        load_luggage_btn.bind(on_press=lambda x: self.switch_to_load_luggage())
        button_layout.add_widget(load_luggage_btn)
        manual_btn = RoundedButton(text="Manual Robot Control", bg_color=THEME_COLORS['primary'],
                                   font_size=50, height=80)
        manual_btn.bind(on_press=lambda x: self.switch_to_manual())
        button_layout.add_widget(manual_btn)
        qr_btn = RoundedButton(text="Scan Boarding Pass", bg_color=THEME_COLORS['primary'],
                               font_size=50, height=80)
        qr_btn.bind(on_press=lambda x: self.switch_to_qr())
        button_layout.add_widget(qr_btn)
        connect_btn = RoundedButton(text="Connect to App", bg_color=THEME_COLORS['primary'],
                                    font_size=50, height=80)
        connect_btn.bind(on_press=lambda x: self.switch_to_connect())
        button_layout.add_widget(connect_btn)
        help_btn = RoundedButton(text="Help", bg_color=THEME_COLORS['accent'],
                                 font_size=50, height=80)
        help_btn.bind(on_press=lambda x: self.switch_to_help())
        button_layout.add_widget(help_btn)
        self.add_widget(button_layout)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

# ------------------ WebSocket Client ------------------
class WebSocketClient:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.status_callback = None
        self.connect()

    def set_status_callback(self, callback):
        self.status_callback = callback
        if self.status_callback:
            self.status_callback(self.connected)

    def connect(self):
        def on_open(ws):
            print("Connected to WebSocket server")
            self.connected = True
            self.reconnect_attempts = 0
            if self.status_callback:
                self.status_callback(True)
        def on_message(ws, message):
            print(f"Message from server: {message}")
        def on_error(ws, error):
            print(f"WebSocket error: {error}")
            self.connected = False
            if self.status_callback:
                self.status_callback(False)
            self.reconnect()
        def on_close(ws, close_status_code, close_msg):
            print("Disconnected from WebSocket server")
            self.connected = False
            if self.status_callback:
                self.status_callback(False)
            self.reconnect()

        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            keep_running=True
        )
        wst = threading.Thread(target=self.ws.run_forever, daemon=True)
        wst.start()

    def reconnect(self):
        if self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            time.sleep(3)
            self.connect()

    def send(self, command):
        if self.ws and self.ws.sock and self.ws.sock.connected:
            try:
                self.ws.send(command)
                print(f"Sent command: {command}")
                return True
            except Exception as e:
                print(f"Error sending command: {e}")
                return False
        else:
            print("WebSocket is not connected.")
            return False

# ------------------ ManualControlScreen ------------------

class ManualControlScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.speed_buttons = {}  # To keep track of speed buttons for highlighting
        self.main_layout = FloatLayout()
        with self.main_layout.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.main_layout.pos, size=self.main_layout.size)
        self.main_layout.bind(pos=self.update_layout, size=self.update_layout)
        header = HeaderBar(title="Manual Robot Control")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        self.server_ip = WS_SERVER_URL
        self.ws_client = WebSocketClient(self.server_ip)
        self.control_grid = self.create_control_grid()
        self.main_layout.add_widget(self.control_grid)
        self.add_widget(self.main_layout)
        # Set default speed to Low Speed on startup and send command "L"
        Clock.schedule_once(lambda dt: self.set_default_speed(), 0)

    def update_layout(self, *args):
        self.bg.pos = self.main_layout.pos
        self.bg.size = self.main_layout.size
        self.control_grid.size_hint = (0.9, 0.7)

    def create_control_grid(self):
        # Increase rows to 4 to add speed control buttons at the bottom.
        grid = GridLayout(rows=4, cols=3, spacing=15, padding=20,
                          size_hint=(0.9, 0.7), pos_hint={'center_x': 0.5, 'center_y': 0.45})
        button_map = {
            # Speed buttons in green
            "Low Speed": ("L", THEME_COLORS['success']),
            "Medium Speed": ("M", THEME_COLORS['success']),
            "High Speed": ("H", THEME_COLORS['success']),
            "Forward": ("w", THEME_COLORS['primary']),
            "Left": ("a", THEME_COLORS['primary']),
            "STOP": ("x", THEME_COLORS['error']),
            "Right": ("d", THEME_COLORS['primary']),
            "Reverse": ("s", THEME_COLORS['primary'])
        }
        layout_structure = [
            ["Low Speed", "Medium Speed", "High Speed"],
            [None, "Forward", None],
            ["Left", "STOP", "Right"],
            [None, "Reverse", None]
        ]
        for row in layout_structure:
            for label in row:
                if label:
                    command, color = button_map[label]
                    if label in ["Low Speed", "Medium Speed", "High Speed"]:
                        btn = RoundedButton(text=label, bg_color=color, font_size=50, height=40, size_hint=(1, 0.5))

                        # Store the speed button reference for highlighting
                        self.speed_buttons[label] = btn
                        # Bind only on_press to keep the speed selection active.
                        btn.bind(on_press=self.create_speed_handler(command, btn))
                    else:
                        btn = RoundedButton(text=label, bg_color=color, font_size=50, height=80)
                        btn.bind(on_press=self.create_press_handler(command))
                        btn.bind(on_release=self.create_release_handler())
                    grid.add_widget(btn)
                else:
                    grid.add_widget(Widget(size_hint=(1, 1)))
        return grid

    def create_speed_handler(self, command, btn):
        def handler(instance):
            self.set_active_speed(btn)
            self.send_command(command)
        return handler

    def set_active_speed(self, active_btn):
        highlight_color = THEME_COLORS['highlight']  # Your highlight color
        normal_color = THEME_COLORS['success']         # The default speed color
        for label, btn in self.speed_buttons.items():
            if btn == active_btn:
                btn.bg_color = highlight_color         # Update bg_color so on_release uses this
                btn.button_color.rgba = highlight_color
            else:
                btn.bg_color = normal_color
                btn.button_color.rgba = normal_color

    def set_default_speed(self):
        if "Low Speed" in self.speed_buttons:
            self.set_active_speed(self.speed_buttons["Low Speed"])
            self.send_command("L")

    def create_press_handler(self, command):
        def handler(instance):
            self.send_command(command)
        return handler

    def create_release_handler(self):
        def handler(instance):
            Clock.schedule_once(lambda dt: self.send_command('x'), 0.1)
        return handler

    def send_command(self, command):
        sound_map = {
            'w': 'move_forward',
            's': 'move_backward',
            'a': 'turn_left',
            'd': 'turn_right'
        }
        if command in sound_map:
            sound_manager.play_sound(sound_map[command], face_widget=self)
        return self.ws_client.send(command) if self.ws_client else False


# ------------------ LoadLuggageScreen ------------------
class LoadLuggageScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.main_layout = FloatLayout()
        with self.main_layout.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.main_layout.pos, size=self.main_layout.size)
        self.main_layout.bind(pos=self.update_layout, size=self.update_layout)
        header = HeaderBar(title="Load Luggage")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        self.server_ip = WS_SERVER_URL
        self.ws_client = WebSocketClient(self.server_ip)
        control_layout = BoxLayout(orientation='vertical', spacing=20, padding=[50, 50],
                                    size_hint=(0.4, 0.4), pos_hint={'center_x': 0.5, 'center_y': 0.5})
        up_btn = RoundedButton(text="Up", bg_color=THEME_COLORS['primary'], font_size=50, height=80)
        down_btn = RoundedButton(text="Down", bg_color=THEME_COLORS['primary'], font_size=50, height=80)
        up_btn.bind(on_press=lambda instance: self.send_command("+"))
        up_btn.bind(on_release=lambda instance: self.send_command("="))
        down_btn.bind(on_press=lambda instance: self.send_command("-"))
        down_btn.bind(on_release=lambda instance: self.send_command("="))
        control_layout.add_widget(up_btn)
        control_layout.add_widget(down_btn)
        self.main_layout.add_widget(control_layout)
        self.add_widget(self.main_layout)

    def update_layout(self, *args):
        self.bg.pos = self.main_layout.pos
        self.bg.size = self.main_layout.size

    def send_command(self, command):
        if self.ws_client:
            return self.ws_client.send(command)
        return False

# ------------------ QR Code Reader Screen ------------------

class QRScreen(Screen):
    def __init__(self, switch_to_postscan, **kwargs):
        super().__init__(**kwargs)
        self.switch_to_postscan = switch_to_postscan
        self.qr_scanned = False
        self.capture = None  # Will hold the OpenCV VideoCapture
        
        main_layout = FloatLayout()
        header = HeaderBar(title="Boarding Pass Scanner")
        header.pos_hint = {'top': 1}
        main_layout.add_widget(header)
        
        camera_container = BoxLayout(
            orientation='vertical',
            size_hint=(0.8, 0.6),
            pos_hint={'center_x': 0.5, 'center_y': 0.5},
            padding=10
        )
        camera_border = BoxLayout(padding=2)
        with camera_border.canvas.before:
            Color(*THEME_COLORS['primary'])
            self.camera_border_rect = Rectangle(pos=camera_border.pos, size=camera_border.size)
        camera_border.bind(pos=self.update_camera_border, size=self.update_camera_border)
        
        # Instead of using Kivy's Camera, we just create an Image widget.
        self.image_display = Image()
        camera_border.add_widget(self.image_display)
        camera_container.add_widget(camera_border)
        main_layout.add_widget(camera_container)
        
        result_card = BoxLayout(
            orientation='vertical',
            size_hint=(0.8, 0.15),
            pos_hint={'center_x': 0.5, 'y': 0.05},
            padding=10
        )
        with result_card.canvas.before:
            Color(*THEME_COLORS['background'])
            self.result_card_rect = Rectangle(pos=result_card.pos, size=result_card.size)
            Color(*THEME_COLORS['primary'])
            self.result_card_border = Line(
                rectangle=(result_card.x, result_card.y, result_card.width, result_card.height),
                width=2
            )
        result_card.bind(pos=self.update_result_card, size=self.update_result_card)
        self.result_label = Label(
            text="Please Scan Your Boarding Pass",
            color=THEME_COLORS['text'],
            font_size=50,
            halign='center',
            valign='middle'
        )
        result_card.add_widget(self.result_label)
        main_layout.add_widget(result_card)
        
        self.add_widget(main_layout)
        # Update texture at 30 FPS
        Clock.schedule_interval(self.update_texture, 1/30)

    def on_pre_enter(self):
        self.qr_scanned = False
        # Open the webcam only when entering the QRScreen.
        self.capture = cv2.VideoCapture(0)
        # Optionally set resolution; here we use 640x640.
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

    def on_leave(self, *args):
        # Release the webcam when leaving to free resources.
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if hasattr(self, 'qr_texture'):
            del self.qr_texture

    def update_camera_border(self, instance, value):
        self.camera_border_rect.pos = instance.pos
        self.camera_border_rect.size = instance.size

    def update_result_card(self, instance, value):
        self.result_card_rect.pos = instance.pos
        self.result_card_rect.size = instance.size
        self.result_card_border.rectangle = (instance.x, instance.y, instance.width, instance.height)

    def update_texture(self, dt):
        # If no capture is available or a QR was already scanned, do nothing.
        if self.capture is None or self.qr_scanned:
            return
        ret, frame = self.capture.read()
        if not ret:
            return
        
        # Rotate the frame by 180 degrees (if needed).
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)
        # Convert the frame from BGR to RGBA for Kivy.
        processed_frame = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2RGBA)
        
        # Attempt to decode the QR code from the processed frame.
        results = decode(processed_frame)
        if results and not self.qr_scanned:
            obj = results[0]
            qr_text = obj.data.decode('utf-8')
            try:
                ticket_info = json.loads(qr_text)
                passenger_name = ticket_info.get("name", "Guest")
                flight_number = ticket_info.get("flight", "Unknown Flight")
                destination = ticket_info.get("to", "Unknown Destination")
                home = ticket_info.get("from", "Unknown Origin")
                dep_time = ticket_info.get("dep_time", "Unknown Boarding Time")
                terminal = ticket_info.get("terminal", "Unknown Terminal")
                gate = ticket_info.get("gate", "Unknown Gate")
                
                payload = {
                    "name": passenger_name,
                    "flight": flight_number,
                    "to": destination,
                    "from": home,
                    "dep_time": dep_time,
                    "terminal": terminal,
                    "gate": gate
                }
                try:
                    response = requests.post(f"{HTTP_SERVER_URL}/api/QR", json=payload, timeout=5)
                    print("Sent QR data:", response.text)
                except Exception as e:
                    print("Error sending QR data:", e)
                self.flash_green_and_transition(passenger_name, flight_number, home, destination, dep_time, terminal, gate)
            except json.JSONDecodeError:
                self.result_label.text = "Invalid QR format. Please try again."
                self.result_label.color = THEME_COLORS['error']
            except Exception as e:
                self.result_label.text = f"Error processing QR: {str(e)}"
                self.result_label.color = THEME_COLORS['error']
        else:
            self.result_label.text = "Please Scan Your Boarding Pass"
            self.result_label.color = THEME_COLORS['text']
        
        # Create a texture from the processed frame.
        h, w, _ = processed_frame.shape
        new_texture = Texture.create(size=(w, h))
        new_texture.blit_buffer(processed_frame.tobytes(), colorfmt='rgba', bufferfmt='ubyte')
        self.image_display.texture = new_texture

    def flash_green_and_transition(self, passenger_name, flight_number, home, destination, dep_time, terminal, gate):
        self.qr_scanned = True
        flash = Widget(size_hint=(1, 1))
        flash.opacity = 1
        with flash.canvas:
            Color(0, 1, 0, 1)
            flash_rect = Rectangle(pos=self.pos, size=self.size)
        flash.bind(
            pos=lambda inst, val: setattr(flash_rect, 'pos', val),
            size=lambda inst, val: setattr(flash_rect, 'size', val)
        )
        self.add_widget(flash)
        anim = Animation(opacity=0, duration=0.5)
        anim.start(flash)
        Clock.schedule_once(lambda dt: self.go_to_postscan(passenger_name, flight_number, home, destination, dep_time, terminal, gate, flash), 0.5)

    def go_to_postscan(self, passenger_name, flight_number, home, destination, dep_time, terminal, gate, flash):
        self.remove_widget(flash)
        postscan_message = (
            f"Welcome, [b]{passenger_name}[/b]!\n"
            f"You're on flight [b]{flight_number}[/b] from [b]{home}[/b] to [b]{destination}[/b]\n"
            f"Boarding at TERMINAL [b]{terminal}[/b] GATE [b]{gate}[/b] at [b]{dep_time}[/b]\n\n"
            f"Would you like Smooth Operator to bring your items to your gate automatically?"
        )
        if self.switch_to_postscan:
            self.switch_to_postscan(postscan_message)
        elif self.manager and self.manager.has_screen("postscan"):
            postscan_screen = self.manager.get_screen("postscan")
            postscan_screen.update_postscan_message(postscan_message)
            self.manager.transition = CardTransition(mode='pop')
            self.manager.current = "postscan"
        else:
            print("PostScanScreen not found")



# ------------------ PostScanScreen ------------------
class PostScanScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.main_layout = FloatLayout()
        header = HeaderBar(title="Scan Complete")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        self.details_label = Label(text="", markup=True, font_size=50,
                                   color=THEME_COLORS['text'], halign='center', valign='middle',
                                   pos_hint={'center_x': 0.5, 'center_y': 0.65})
        self.main_layout.add_widget(self.details_label)
        button_layout = BoxLayout(orientation='horizontal', size_hint=(0.5, 0.1),
                                  pos_hint={'center_x': 0.5, 'y': 0.3}, spacing=20)
        yes_button = RoundedButton(text="Yes", bg_color=THEME_COLORS['success'], font_size=50)
        no_button = RoundedButton(text="No", bg_color=THEME_COLORS['error'], font_size=50)
        yes_button.bind(on_press=self.on_yes)
        no_button.bind(on_press=self.on_no)
        button_layout.add_widget(yes_button)
        button_layout.add_widget(no_button)
        self.main_layout.add_widget(button_layout)
        self.add_widget(self.main_layout)

    def update_postscan_message(self, message):
        self.details_label.text = message

    def on_yes(self, instance):
        app = App.get_running_app()
        app.sm.transition = CardTransition(mode='pop')
        app.sm.current = "face"

    def on_no(self, instance):
        self.qr_scanned = False
        app = App.get_running_app()
        app.sm.transition = CardTransition(mode='pop')
        app.sm.current = "menu"

# ------------------ HelpScreen ------------------
class HelpScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.main_layout = FloatLayout()
        header = HeaderBar(title="Help")
        header.pos_hint = {'top': 1}
        self.main_layout.add_widget(header)
        instructions = (
            "[b]How to Use SmoothOperator:[/b]\n\n"
            "1. From the [b]Face[/b] screen, tap the screen to enter the menu.\n"
            "2. Select [b]Manual Robot Control[/b] to control the robot manually using the on-screen buttons.\n"
            "3. Select [b]Scan Boarding Pass[/b] to validate your boarding pass using the QR scanner.\n"
            "4. Follow the prompts on each screen for further instructions.\n"
            "5. Use the [b]Back[/b] button in the header to navigate to the previous screen.\n\n"
            "For additional support, please contact our support team."
        )
        help_label = Label(text=instructions, markup=True, font_size=40,
                           color=THEME_COLORS['text'], halign='center', valign='middle',
                           pos_hint={'center_x': 0.5, 'center_y': 0.5}, size_hint=(0.8, 0.6))
        help_label.bind(size=help_label.setter('text_size'))
        self.main_layout.add_widget(help_label)
        self.add_widget(self.main_layout)


# ------------------ SoundManager ------------------
class SoundManager:
    def __init__(self):
        pygame.mixer.init()
        self.current_audio = None
        self.audio_files = {
            'start': ["PythonUI/audio/Hi_Im_SmoothOperator.mp3"],
            'Connect': ["PythonUI/audio/Connect.mp3"],
            'Help': ["PythonUI/audio/Help.mp3"],
            'ManualControl': ["PythonUI/audio/ManualControl.mp3"],
            'Menu': ["PythonUI/audio/Menu.mp3"],
            'stop': ["PythonUI/audio/Scan.mp3"],
            'ScanSuccess': ["PythonUI/audio/ScanSuccess.mp3"],
            'BEEP': ["PythonUI/audio/BEEPBEEP.mp3"],
            'Watchout': ["PythonUI/audio/Watchout.mp3"],
            'Luggage': ["PythonUI/audio/Luggage.mp3"],
        }

    def play_sound(self, sound_key, face_widget=None):
        if sound_key in self.audio_files:
            sound_file = random.choice(self.audio_files[sound_key])
            try:
                if pygame.mixer.music.get_busy():
                    pygame.mixer.music.stop()
                pygame.mixer.music.load(sound_file)
                pygame.mixer.music.play()
                if face_widget:
                    face_widget.start_mouth_animation()
                    # Only schedule if not already scheduled
                    if not hasattr(face_widget, '_mouth_check_event') or face_widget._mouth_check_event is None:
                        face_widget._mouth_check_event = Clock.schedule_interval(face_widget.check_audio_finished, 0.1)
            except Exception as e:
                print(f"Error playing sound {sound_file}: {e}")
        else:
            print(f"Invalid sound key: {sound_key}")

# Initialize a global sound manager
sound_manager = SoundManager()

# ------------------ Main Application ------------------
class SmoothOperatorApp(App):
    def build(self):
        self.title = "SmoothOperator"
        self.sm = ScreenManager(transition=FadeTransition(duration=0.5))
        face_screen = Screen(name="face")
        menu_screen = Screen(name="menu")
        manual_screen = ManualControlScreen(name="manual")
        qr_screen = QRScreen(switch_to_postscan=self.switch_to_postscan, name="qr")
        postscan_screen = PostScanScreen(name="postscan")
        help_screen = HelpScreen(name="help")
        connect_screen = ConnectScreen(name="connect")
        load_luggage_screen = LoadLuggageScreen(name="load_luggage")

        # Instantiate FaceScreen and keep a reference to it
        self.face_widget = FaceScreen(self.switch_to_menu)
        face_screen.add_widget(self.face_widget)

        menu_widget = MenuScreen(switch_to_manual=self.switch_to_manual,
                                switch_to_qr=self.switch_to_qr,
                                switch_to_help=self.switch_to_help,
                                switch_to_connect=self.switch_to_connect,
                                switch_to_load_luggage=self.switch_to_load_luggage)
        menu_screen.add_widget(menu_widget)

        self.sm.add_widget(face_screen)
        self.sm.add_widget(menu_screen)
        self.sm.add_widget(manual_screen)
        self.sm.add_widget(qr_screen)
        self.sm.add_widget(postscan_screen)
        self.sm.add_widget(help_screen)
        self.sm.add_widget(connect_screen)
        self.sm.add_widget(load_luggage_screen)

        self.sm.current = "face"
        # Now pass the actual FaceScreen instance to play_sound
        sound_manager.play_sound('start', face_widget=self.face_widget)
        return self.sm


    def switch_to_menu(self):
        sound_manager.play_sound('Menu')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "menu"

    def switch_to_manual(self):
        sound_manager.play_sound('ManualControl')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "manual"

    def switch_to_qr(self):
        sound_manager.play_sound('Scan')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "qr"

    def switch_to_help(self):
        sound_manager.play_sound('Help')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "help"

    def switch_to_connect(self):
        sound_manager.play_sound('Connect')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "connect"

    def switch_to_load_luggage(self):
        sound_manager.play_sound('Luggage')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "load_luggage"

    def switch_to_postscan(self, message=None):
        if message:
            postscan_screen = self.sm.get_screen("postscan")
            postscan_screen.update_postscan_message(message)
        sound_manager.play_sound('ScanSuccess')
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "postscan"

if __name__ == '__main__':
    SmoothOperatorApp().run()

