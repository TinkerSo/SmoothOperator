import threading
import time
import websocket
# import cv2
# import numpy as np
# from pyzbar.pyzbar import decode

from kivy.app import App
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Ellipse, Color
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.uix.camera import Camera


# ------------------ FaceScreen and MenuScreen ------------------

class FaceScreen(Widget):
    def __init__(self, switch_screen_callback, **kwargs):
        super().__init__(**kwargs)
        self.switch_screen_callback = switch_screen_callback
        self.blinking = False
        self.eye_size = (120, 200)  # Slightly larger eyes
        self.default_eye_pos = {'left': (-120, 0), 'right': (40, 0)}
        
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        if self._keyboard:
            self._keyboard.bind(on_key_down=self.on_key_down)
            self._keyboard.bind(on_key_up=self.on_key_up)
        
        with self.canvas:
            Color(0, 191/255, 255/255)
            self.left_eye = Ellipse(size=self.eye_size)
            self.right_eye = Ellipse(size=self.eye_size)
        
        self.update_positions()
        self.bind(pos=self.update_positions, size=self.update_positions)
        Clock.schedule_interval(self.blink, 3)
    
    def _keyboard_closed(self):
        if self._keyboard:
            self._keyboard.unbind(on_key_down=self.on_key_down)
            self._keyboard.unbind(on_key_up=self.on_key_up)
            self._keyboard = None
    
    def update_positions(self, *args):
        self.left_eye.pos = (
            self.center_x + self.default_eye_pos['left'][0],
            self.center_y + self.default_eye_pos['left'][1]
        )
        self.right_eye.pos = (
            self.center_x + self.default_eye_pos['right'][0],
            self.center_y + self.default_eye_pos['right'][1]
        )
    
    def blink(self, dt):
        if not self.blinking:
            self.left_eye.size = (120, 20)
            self.right_eye.size = (120, 20)
            self.blinking = True
            Clock.schedule_once(self.unblink, 0.2)
    
    def unblink(self, dt):
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.blinking = False
    
    def on_touch_down(self, touch):
        # Switch to menu screen when touched
        self.switch_screen_callback()
    
    def on_key_down(self, keyboard, keycode, text, modifiers):
        movement_x = 80
        movement_y = 50
        
        movements = {
            'w': (0, movement_y),
            's': (0, -movement_y),
            'a': (-movement_x, 0),
            'd': (movement_x, 0)
        }
        
        if keycode[1] in movements:
            dx, dy = movements[keycode[1]]
            self.default_eye_pos['left'] = (-120 + dx, dy)
            self.default_eye_pos['right'] = (40 + dx, dy)
            self.update_positions()
    
    def on_key_up(self, keyboard, keycode):
        self.default_eye_pos = {'left': (-120, 0), 'right': (40, 0)}
        self.update_positions()


class MenuScreen(BoxLayout):
    def __init__(self, switch_to_manual, switch_to_qr, **kwargs):
        super().__init__(orientation='vertical', **kwargs)
        self.add_widget(
            Button(
                text="Manual Control",
                font_size=24,
                on_press=lambda x: switch_to_manual()
            )
        )
        self.add_widget(
            Button(
                text="Scan QR Code",
                font_size=24,
                on_press=lambda x: switch_to_qr()
            )
        )


# ------------------ Manual Control Screen ------------------

class WebSocketClient:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.retry_count = 0
        self.max_retries = 5
        self.connect()

    def connect(self):
        def on_open(ws):
            print("Connected to WebSocket server")
            self.retry_count = 0

        def on_message(ws, message):
            print("Message from server:", message)

        def on_error(ws, error):
            print("WebSocket error:", error)

        def on_close(ws, close_status_code, close_msg):
            print("Disconnected from WebSocket server")
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                print(
                    f"Retrying connection in 3 seconds (Attempt {self.retry_count}/{self.max_retries})"
                )
                time.sleep(3)
                self.connect()
            else:
                print("WebSocket connection failed after multiple attempts.")

        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )
        # Run the WebSocket in a separate thread so it doesn't block the Kivy UI
        wst = threading.Thread(target=self.ws.run_forever, daemon=True)
        wst.start()

    def send(self, command):
        try:
            if self.ws and self.ws.sock and self.ws.sock.connected:
                self.ws.send(command)
                print(f"Sent command: {command}")
            else:
                print("WebSocket is not connected.")
        except Exception as e:
            print("Error sending command:", e)


class ManualControlScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Set your server IP address here (update as needed)
        self.server_ip = "ws://10.192.31.229:3000"
        self.ws_client = WebSocketClient(self.server_ip)

        # Create a 3x3 grid layout for manual controls
        grid = GridLayout(rows=3, cols=3, spacing=10, padding=10)

        font_sz = 40
        btn_size_hint = (1, 1)  # Each button takes full cell space

        # Row 1: Top-left, Top-center, Top-right
        btn_top_left = Button(
            text="",
            font_size=font_sz,
            background_color=(0, 0, 0, 1),
            disabled=True,
            size_hint=btn_size_hint
        )
        grid.add_widget(btn_top_left)

        btn_up = Button(
            text="Forward",
            font_size=font_sz,
            size_hint=btn_size_hint
        )
        btn_up.bind(
            on_press=lambda x: self.handle_press_in('w'),
            on_release=lambda x: self.handle_press_out()
        )
        grid.add_widget(btn_up)

        btn_top_right = Button(
            text="",
            font_size=font_sz,
            background_color=(0, 0, 0, 1),
            disabled=True,
            size_hint=btn_size_hint
        )
        grid.add_widget(btn_top_right)

        # Row 2: Middle-left, Middle-center, Middle-right
        btn_left = Button(
            text="Left",
            font_size=font_sz,
            size_hint=btn_size_hint
        )
        btn_left.bind(
            on_press=lambda x: self.handle_press_in('a'),
            on_release=lambda x: self.handle_press_out()
        )
        grid.add_widget(btn_left)

        btn_stop = Button(
            text="Stop",
            font_size=font_sz,
            background_color=(1, 0, 0, 1),
            size_hint=btn_size_hint
        )
        btn_stop.bind(on_press=lambda x: self.send_command('x'))
        grid.add_widget(btn_stop)

        btn_right = Button(
            text="Right",
            font_size=font_sz,
            size_hint=btn_size_hint
        )
        btn_right.bind(
            on_press=lambda x: self.handle_press_in('d'),
            on_release=lambda x: self.handle_press_out()
        )
        grid.add_widget(btn_right)

        # Row 3: Bottom-left, Bottom-center, Bottom-right
        btn_bottom_left = Button(
            text="",
            font_size=font_sz,
            background_color=(0, 0, 0, 1),
            disabled=True,
            size_hint=btn_size_hint
        )
        grid.add_widget(btn_bottom_left)

        btn_down = Button(
            text="Reverse",
            font_size=font_sz,
            size_hint=btn_size_hint
        )
        btn_down.bind(
            on_press=lambda x: self.handle_press_in('s'),
            on_release=lambda x: self.handle_press_out()
        )
        grid.add_widget(btn_down)

        btn_bottom_right = Button(
            text="",
            font_size=font_sz,
            background_color=(0, 0, 0, 1),
            disabled=True,
            size_hint=btn_size_hint
        )
        grid.add_widget(btn_bottom_right)

        self.add_widget(grid)

    def send_command(self, command):
        if self.ws_client:
            self.ws_client.send(command)

    def handle_press_in(self, command):
        self.send_command(command)

    def handle_press_out(self):
        # Send stop command when the button is released
        self.send_command('x')


# ------------------ QR Code Reader Screen ------------------

class QRScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Create a vertical layout for the camera and result label
        layout = BoxLayout(orientation='vertical')
        
    #     # Create and add the camera widget.
    #     # Adjust the resolution as needed.
    #     self.camera = Camera(play=True, resolution=(640, 480))
    #     layout.add_widget(self.camera)
        
    #     # Create a label to display QR code results.
    #     self.result_label = Label(
    #         text="Point the camera at a QR code",
    #         size_hint=(1, 0.2),
    #         font_size=24
    #     )
    #     layout.add_widget(self.result_label)
        
    #     self.add_widget(layout)
        
    #     # Schedule a periodic callback to decode QR codes from the camera feed.
    #     Clock.schedule_interval(self.decode_qr, 1/5.0)  # 5 times per second

    # def decode_qr(self, dt):
    #     if not self.camera.texture:
    #         return
        
    #     # Retrieve the texture from the camera widget.
    #     texture = self.camera.texture
    #     # Get the pixel data as bytes and convert to a NumPy array.
    #     buf = texture.pixels  # RGBA data
    #     data = np.frombuffer(buf, np.uint8)
    #     data = data.reshape(texture.height, texture.width, 4)
        
    #     # Convert the image from RGBA to grayscale using OpenCV.
    #     gray = cv2.cvtColor(data, cv2.COLOR_RGBA2GRAY)
        
    #     # Use pyzbar to decode any QR codes in the grayscale image.
    #     results = decode(gray)
        
    #     if results:
    #         # If a QR code is found, display its data.
    #         qr_data = results[0].data.decode('utf-8')
    #         self.result_label.text = "QR Code: " + qr_data
    #     else:
    #         self.result_label.text = "No QR Code detected"


# ------------------ Main Application ------------------

class SmoothOperatorApp(App):
    def build(self):
        self.sm = ScreenManager()

        face_screen = Screen(name="face")
        menu_screen = Screen(name="menu")
        manual_screen = ManualControlScreen(name="manual")
        qr_screen = QRScreen(name="qr")

        # Face screen with FaceScreen widget
        face_widget = FaceScreen(self.switch_to_menu)
        face_screen.add_widget(face_widget)

        # Menu screen with MenuScreen widget
        menu_widget = MenuScreen(self.switch_to_manual, self.switch_to_qr)
        menu_screen.add_widget(menu_widget)

        self.sm.add_widget(face_screen)
        self.sm.add_widget(menu_screen)
        self.sm.add_widget(manual_screen)
        self.sm.add_widget(qr_screen)

        return self.sm

    def switch_to_menu(self):
        self.sm.current = "menu"

    def switch_to_manual(self):
        self.sm.current = "manual"

    def switch_to_qr(self):
        self.sm.current = "qr"


if __name__ == '__main__':
    SmoothOperatorApp().run()
