import threading
import time
import websocket
import cv2
import numpy as np
from pyzbar.pyzbar import decode

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
from kivy.graphics.texture import Texture
from kivy.graphics.context_instructions import Rotate, PushMatrix, PopMatrix


# ------------------ FaceScreen and MenuScreen ------------------
class FaceScreen(Widget):
    def __init__(self, switch_screen_callback, **kwargs):
        super().__init__(**kwargs)
        self.switch_screen_callback = switch_screen_callback
        self.blinking = False

        # Adjust eye size and spacing relative to the window size
        self.eye_size = (Window.width * 0.25, Window.height * 0.5)  # 25% of width, 50% of height
        self.eye_spacing = Window.width * 0.5  # Space between eyes

        # Eye movement offsets
        self.eye_offset_x = 0
        self.eye_offset_y = 0

        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        if self._keyboard:
            self._keyboard.bind(on_key_down=self.on_key_down)
            self._keyboard.bind(on_key_up=self.on_key_up)

        with self.canvas:
            Color(0, 191/255, 255/255)
            self.left_eye = Ellipse(size=self.eye_size)
            self.right_eye = Ellipse(size=self.eye_size)

        self.update_positions()
        self.bind(pos=self.update_positions, size=self.on_resize)
        Clock.schedule_interval(self.blink, 3)

    def _keyboard_closed(self):
        if self._keyboard:
            self._keyboard.unbind(on_key_down=self.on_key_down)
            self._keyboard.unbind(on_key_up=self.on_key_up)
            self._keyboard = None

    def update_positions(self, *args):
        """ Ensure eyes are correctly positioned and spaced properly. """
        left_x = self.center_x - self.eye_spacing / 2 - self.eye_size[0] / 2 + self.eye_offset_x
        right_x = self.center_x + self.eye_spacing / 2 - self.eye_size[0] / 2 + self.eye_offset_x

        left_y = self.center_y - self.eye_size[1] / 2 + self.eye_offset_y
        right_y = self.center_y - self.eye_size[1] / 2 + self.eye_offset_y

        self.left_eye.pos = (left_x, left_y)
        self.right_eye.pos = (right_x, right_y)

    def on_resize(self, *args):
        """ Recalculate size and position dynamically when window resizes. """
        self.eye_size = (Window.width * 0.25, Window.height * 0.5)  # Keep eyes proportionally large
        self.eye_spacing = Window.width * 0.5  # Maintain spacing

        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size

        self.update_positions()

    def on_key_down(self, keyboard, keycode, text, modifiers):
        """ Move eyes based on keyboard input. """
        movement_x = 30  # Move horizontally
        movement_y = 20  # Move vertically

        if keycode[1] == 'w':  # Move Up
            self.eye_offset_y += movement_y
        elif keycode[1] == 's':  # Move Down
            self.eye_offset_y -= movement_y
        elif keycode[1] == 'a':  # Move Left
            self.eye_offset_x -= movement_x
        elif keycode[1] == 'd':  # Move Right
            self.eye_offset_x += movement_x

        self.update_positions()

    def on_key_up(self, keyboard, keycode):
        """ Reset eyes to default position when key is released. """
        self.eye_offset_x = 0
        self.eye_offset_y = 0
        self.update_positions()

    def blink(self, dt):
        """ Simulate blinking by reducing eye height. """
        if not self.blinking:
            self.left_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.right_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.blinking = True
            Clock.schedule_once(self.unblink, 0.2)

    def unblink(self, dt):
        """ Restore eyes to original size after blinking. """
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.blinking = False

    def on_touch_down(self, touch):
        """ Switch to the menu screen when the screen is tapped. """
        self.switch_screen_callback()

# ------------------ MenuScreen ------------------
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

from kivy.graphics.texture import Texture
from kivy.uix.image import Image
from kivy.clock import Clock

class QRScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Create a vertical layout
        layout = BoxLayout(orientation='vertical')

        # Create the Camera widget
        self.camera = Camera(play=True, resolution=(640, 480))
        self.camera.allow_stretch = True  # Ensure the camera resizes properly
        # layout.add_widget(self.camera)

        # Create an Image widget to display the rotated camera texture
        self.image_display = Image()
        layout.add_widget(self.image_display)

        # Create a label to display QR code results
        self.result_label = Label(
            text="Point the camera at a QR code",
            size_hint=(1, 0.2),
            font_size=24
        )
        layout.add_widget(self.result_label)

        self.add_widget(layout)

        # Schedule a function to update the camera feed with rotation
        Clock.schedule_interval(self.update_texture, 1/30)  # 30 FPS

    def update_texture(self, dt):
        if not self.camera.texture:
            return

        # Get the camera texture and convert to OpenCV format
        texture = self.camera.texture
        w, h = texture.size
        pixels = texture.pixels  # Get raw pixel data

        # Convert the raw data to a NumPy array
        frame = np.frombuffer(pixels, np.uint8).reshape(h, w, 4)  # RGBA format

        # Rotate the image 90 degrees counterclockwise
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Decode any QR codes in the rotated frame
        results = decode(rotated_frame)
        if results:
            # Display the first decoded QR code result
            qr_text = results[0].data.decode('utf-8')
            self.result_label.text = f"QR Code: {qr_text}"
        else:
            self.result_label.text = "Point the camera at a QR code"

        # Create a new Kivy texture and apply it to the image widget
        new_texture = Texture.create(size=(h, w))  # Swap w and h after rotation
        new_texture.blit_buffer(rotated_frame.tobytes(), colorfmt='rgba', bufferfmt='ubyte')
        self.image_display.texture = new_texture



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
