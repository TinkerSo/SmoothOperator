import threading
import time
import websocket
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import json
import pyttsx3

from kivy.app import App
from kivy.uix.screenmanager import Screen, ScreenManager, FadeTransition, SlideTransition, CardTransition
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
from kivy.graphics.context_instructions import Rotate, PushMatrix, PopMatrix
from kivy.uix.behaviors import ButtonBehavior
from kivy.utils import get_color_from_hex
from kivy.animation import Animation

# Set app theme colors
THEME_COLORS = {
    'primary': get_color_from_hex('#2196F3'),         # Blue
    'secondary': get_color_from_hex('#03A9F4'),       # Light Blue
    'accent': get_color_from_hex('#FF9800'),          # Orange
    'background': get_color_from_hex('#FFFFFF'),      # White
    'text': get_color_from_hex('#212121'),            # Dark Grey
    'success': get_color_from_hex('#4CAF50'),         # Green
    'error': get_color_from_hex('#F44336'),           # Red
    'disabled': get_color_from_hex('#BDBDBD')         # Light Grey
}

# Set default window size
Window.size = (800, 600)
Window.clearcolor = THEME_COLORS['background']

# ------------------ Custom Widgets ------------------
class RoundedButton(ButtonBehavior, BoxLayout):
    def __init__(self, text="", icon=None, bg_color=THEME_COLORS['primary'], text_color=(1, 1, 1, 1), 
                 font_size=50, radius=20, size_hint=(1, 1), height=60, **kwargs):
        super().__init__(size_hint=size_hint, height=height, **kwargs)
        self.bg_color = bg_color
        self.text = text
        self.radius = radius
        
        # Create content layout
        content = BoxLayout(orientation='horizontal', padding=10, spacing=10)
        
        # Add icon if provided
        if icon:
            icon_img = Image(source=icon, size_hint=(None, None), size=(30, 30))
            content.add_widget(icon_img)
        
        # Add text label
        label = Label(
            text=text,
            font_size=font_size,
            color=text_color,
            halign='center',
            valign='middle',
            size_hint=(1, 1)
        )
        content.add_widget(label)
        
        self.add_widget(content)
        
        # Draw rounded background
        with self.canvas.before:
            Color(*bg_color)
            self.bg_rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[self.radius])
        
        self.bind(pos=self.update_rect, size=self.update_rect)
        
    def update_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size
        
    def on_press(self):
        with self.canvas.before:
            Color(*(self.bg_color[0] * 0.8, self.bg_color[1] * 0.8, self.bg_color[2] * 0.8, self.bg_color[3]))
            self.bg_rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[self.radius])
            
    def on_release(self):
        with self.canvas.before:
            Color(*self.bg_color)
            self.bg_rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[self.radius])


class HeaderBar(BoxLayout):
    def __init__(self, title="SmoothOperatort", icon=None, **kwargs):
        super().__init__(orientation='horizontal', size_hint=(1, None), height=60, **kwargs)
        
        with self.canvas.before:
            Color(*THEME_COLORS['primary'])
            self.rect = Rectangle(pos=self.pos, size=self.size)
        
        self.bind(pos=self.update_rect, size=self.update_rect)
        
        # Create header content
        content = BoxLayout(orientation='horizontal', padding=10, spacing=10)
        
        # Add icon if provided
        if icon:
            icon_img = Image(source=icon, size_hint=(None, None), size=(40, 40))
            content.add_widget(icon_img)
        
        # Add title label
        title_label = Label(
            text=title,
            font_size=50,
            color=(1, 1, 1, 1),
            halign='left',
            valign='middle',
            size_hint=(1, 1),
            text_size=(None, None)
        )
        content.add_widget(title_label)
        
        # Add back button
        back_button = Button(
            text="Back",
            size_hint=(None, None),
            size=(80, 40),
            background_color=THEME_COLORS['secondary'],
            font_size=20
        )
        back_button.bind(on_press=self.go_back)
        content.add_widget(back_button)
        
        self.add_widget(content)
    
    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size
        
    def go_back(self, instance):
        app = App.get_running_app()
        # If we're on the menu screen, go back to face screen
        if app.sm.current == "menu":
            app.sm.transition = CardTransition(mode='pop')
            app.sm.current = "face"
        # If we're on any other screen, go back to menu
        elif app.sm.current != "face":
            app.sm.transition = CardTransition(mode='pop')
            app.sm.current = "menu"


# ------------------ FaceScreen ------------------
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

        # Set background color
        with self.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.pos, size=self.size)

        self.bind(pos=self.update_bg, size=self.update_bg)

        # Initialize keyboard controls
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        if self._keyboard:
            self._keyboard.bind(on_key_down=self.on_key_down)
            self._keyboard.bind(on_key_up=self.on_key_up)

        # Draw eyes
        with self.canvas:
            Color(*THEME_COLORS['primary'])  # Eye color
            self.left_eye = Ellipse(size=self.eye_size)
            self.right_eye = Ellipse(size=self.eye_size)

        self.update_positions()
        self.bind(pos=self.update_positions, size=self.on_resize)
        Clock.schedule_interval(self.blink, 3)

    def speak_phrase(self):
        """Speak the phrase 'Excuse me, coming through' with a cute voice."""
        def run_tts():
            engine = pyttsx3.init()
            # Adjust the rate and volume for a "cute" tone
            engine.setProperty('rate', 150)  # slower rate may sound cuter
            engine.setProperty('volume', 1)  # max volume
            # Optionally choose a female voice if available
            voices = engine.getProperty('voices')
            for voice in voices:
                if 'female' in voice.name.lower():
                    engine.setProperty('voice', voice.id)
                    break
            engine.say("Excuse me, coming through")
            engine.runAndWait()
        threading.Thread(target=run_tts, daemon=True).start()

    def on_touch_down(self, touch):
        """When the screen is tapped, speak the phrase then switch screen."""
        self.speak_phrase()
        # Schedule the screen transition slightly delayed to let the voice start
        Clock.schedule_once(lambda dt: self.switch_screen_callback(), 1.0)
        return super().on_touch_down(touch)
    
    def _keyboard_closed(self):
        """ Unbind keyboard events when the keyboard is closed. """
        if self._keyboard:
            self._keyboard.unbind(on_key_down=self.on_key_down)
            self._keyboard.unbind(on_key_up=self.on_key_up)
            self._keyboard = None

    def update_bg(self, *args):
        """ Ensure the background resizes properly. """
        self.bg.pos = self.pos
        self.bg.size = self.size

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
        movement_x = 300  # Move horizontally
        movement_y = 200  # Move vertically

        if keycode[1] == 'w':  # Move Up
            self.eye_offset_y = movement_y
        elif keycode[1] == 's':  # Move Down
            self.eye_offset_y = -movement_y
        elif keycode[1] == 'a':  # Move Left
            self.eye_offset_x = -movement_x
        elif keycode[1] == 'd':  # Move Right
            self.eye_offset_x = movement_x

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
class MenuScreen(FloatLayout):
    def __init__(self, switch_to_manual, switch_to_qr, **kwargs):
        super().__init__(**kwargs)
        
        # Set background with airport theme
        with self.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.pos, size=self.size)
        
        self.bind(pos=self.update_bg, size=self.update_bg)
        
        # Add header
        header = HeaderBar(title="Menu")
        header.pos_hint = {'top': 1}
        self.add_widget(header)
        
        # Create menu buttons container
        button_layout = BoxLayout(
            orientation='vertical',
            spacing=20,
            padding=[50, 50],
            size_hint=(0.8, 0.5),
            pos_hint={'center_x': 0.5, 'center_y': 0.45}
        )
        
        # Manual Control Button
        manual_btn = RoundedButton(
            text="Manual Robot Control",
            bg_color=THEME_COLORS['primary'],
            font_size=50,
            height=80
        )
        manual_btn.bind(on_press=lambda x: switch_to_manual())
        button_layout.add_widget(manual_btn)
        
        # QR Code Button
        qr_btn = RoundedButton(
            text="Scan Boarding Pass",
            bg_color=THEME_COLORS['accent'],
            font_size=50,
            height=80
        )
        qr_btn.bind(on_press=lambda x: switch_to_qr())
        button_layout.add_widget(qr_btn)
        
        self.add_widget(button_layout)
    
    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size


# ------------------ WebSocket Client ------------------
class WebSocketClient:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.retry_count = 0
        self.max_retries = 5
        self.connected = False
        self.status_callback = None
        self.connect()

    def set_status_callback(self, callback):
        self.status_callback = callback
        # Call immediately with current status
        if self.status_callback:
            self.status_callback(self.connected)

    def connect(self):
        def on_open(ws):
            print("Connected to WebSocket server")
            self.connected = True
            self.retry_count = 0
            if self.status_callback:
                self.status_callback(True)

        def on_message(ws, message):
            print("Message from server:", message)

        def on_error(ws, error):
            print("WebSocket error:", error)
            self.connected = False
            if self.status_callback:
                self.status_callback(False)

        def on_close(ws, close_status_code, close_msg):
            print("Disconnected from WebSocket server")
            self.connected = False
            if self.status_callback:
                self.status_callback(False)
                
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
                return True
            else:
                print("WebSocket is not connected.")
                return False
        except Exception as e:
            print("Error sending command:", e)
            return False


# ------------------ Manual Control Screen ------------------
class ManualControlScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Main layout
        self.main_layout = FloatLayout()
        
        # Set background
        with self.main_layout.canvas.before:
            Color(*THEME_COLORS['background'])
            self.bg = Rectangle(pos=self.main_layout.pos, size=self.main_layout.size)

        self.main_layout.bind(pos=self.update_layout, size=self.update_layout)

        # Add header
        header = HeaderBar(title="Manual Robot Control")
        header.pos_hint = {'top': 1}  # ✅ Ensure it is positioned at the top
        self.main_layout.add_widget(header)

        # Create the status indicator
        self.status_icon, self.status_circle = self.create_status_indicator()
        self.main_layout.add_widget(self.status_icon)

        # WebSocket setup AFTER status_icon is created
        self.server_ip = "ws://10.192.31.229:3000"
        self.ws_client = WebSocketClient(self.server_ip)
        self.ws_client.set_status_callback(self.update_connection_status)  

        # Control grid
        self.control_grid = self.create_control_grid()
        self.main_layout.add_widget(self.control_grid)

        self.add_widget(self.main_layout)

        # Schedule connection status updates
        Clock.schedule_interval(self.check_connection, 5)

    def update_layout(self, *args):
        """Force updates to the layout when the window resizes."""
        self.bg.pos = self.main_layout.pos
        self.bg.size = self.main_layout.size
        self.control_grid.size_hint = (0.9, 0.6)  # Re-apply layout constraints


    def create_status_indicator(self):
        """Creates and returns the status indicator widget with dynamic updates."""
        status_icon = Widget(size_hint=(None, None), size=(20, 20), pos_hint={'center_x': 0.5, 'top': 0.93})
        
        with status_icon.canvas:
            Color(0.8, 0.2, 0.2, 1)  # Default Red (Disconnected)
            status_circle = Ellipse(pos=status_icon.pos, size=status_icon.size)

        # Bind position and size updates so the ellipse follows the widget
        def update_status_circle(instance, value):
            status_circle.pos = status_icon.pos
            status_circle.size = status_icon.size

        status_icon.bind(pos=update_status_circle, size=update_status_circle)

        return status_icon, status_circle



    def create_control_grid(self):
        """ Creates and returns the control button grid. """
        grid = GridLayout(rows=3, cols=3, spacing=15, padding=20, size_hint=(0.9, 0.6), pos_hint={'center_x': 0.5, 'center_y': 0.45})

        button_map = {
            "Forward": ("w", THEME_COLORS['primary']),
            "Left": ("a", THEME_COLORS['primary']),
            "STOP": ("x", THEME_COLORS['error']),
            "Right": ("d", THEME_COLORS['primary']),
            "Reverse": ("s", THEME_COLORS['primary']),
        }

        layout_structure = [
            [None, "Forward", None],
            ["Left", "STOP", "Right"],
            [None, "Reverse", None]
        ]

        for row in layout_structure:
            for label in row:
                if label:
                    command, color = button_map[label]
                    btn = RoundedButton(text=label, bg_color=color, font_size=50, height=80)
                    if label == "STOP":
                        btn.bind(on_press=lambda x: self.send_command(command))
                    else:
                        btn.bind(on_press=lambda x, cmd=command: self.handle_press_in(cmd))
                        btn.bind(on_release=lambda x: self.handle_press_out())
                    grid.add_widget(btn)
                else:
                    grid.add_widget(Widget(size_hint=(1, 1)))  # Empty placeholder
        return grid

    def update_connection_status(self, connected):
        """ Updates the status dot color based on connection status. """
        color = THEME_COLORS['success'] if connected else THEME_COLORS['error']
        with self.status_icon.canvas:
            Color(*color)
            self.status_circle.pos = self.status_icon.pos
            self.status_circle.size = self.status_icon.size

    def check_connection(self, dt):
        """ Checks the WebSocket connection status periodically. """
        self.update_connection_status(self.ws_client.connected)

    def send_command(self, command):
        """ Sends a command via WebSocket. """
        return self.ws_client.send(command) if self.ws_client else False

    def handle_press_in(self, command):
        """ Sends a movement command when a button is pressed. """
        self.send_command(command)

    def handle_press_out(self):
        """ Stops the movement when a button is released. """
        self.send_command('x')


# ------------------ QR Code Reader Screen ------------------
class QRScreen(Screen):
    def __init__(self, switch_to_postscan, **kwargs):
        super().__init__(**kwargs)
        self.switch_to_postscan = switch_to_postscan
        self.qr_scanned = False
        main_layout = FloatLayout()
        
        header = HeaderBar(title="Boarding Pass Scanner")
        header.pos_hint = {'top': 1}
        main_layout.add_widget(header)
        
        instructions = Label(
            text="Scan your boarding pass QR code",
            font_size=50,
            color=THEME_COLORS['text'],
            pos_hint={'center_x': 0.5, 'top': 0.9}
        )
        main_layout.add_widget(instructions)

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
        
        self.camera = Camera(play=True, resolution=(640, 480), index=1)
        self.camera.allow_stretch = True
        
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
            self.result_card_border = Line(rectangle=(result_card.x, result_card.y, result_card.width, result_card.height), width=2)
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
        
        Clock.schedule_interval(self.update_texture, 1/30)
    def on_pre_enter(self):
        # Reset the scanning flag when the screen is shown again.
        self.qr_scanned = False
    def update_camera_border(self, instance, value):
        self.camera_border_rect.pos = instance.pos
        self.camera_border_rect.size = instance.size
        
    def update_result_card(self, instance, value):
        self.result_card_rect.pos = instance.pos
        self.result_card_rect.size = instance.size
        self.result_card_border.rectangle = (instance.x, instance.y, instance.width, instance.height)

    def update_texture(self, dt):
        if not self.camera.texture or self.qr_scanned:
            return

        texture = self.camera.texture
        w, h = texture.size
        pixels = texture.pixels
        frame = np.frombuffer(pixels, np.uint8).reshape(h, w, 4)
        
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h_rotated, w_rotated = rotated_frame.shape[:2]
        center_x, center_y = w_rotated // 2, h_rotated // 2
        qr_size = min(w_rotated, h_rotated) // 2
        corner_size = 30
        line_thickness = 2
        corner_color = (0, 191, 255, 255)
        
        cv2.line(rotated_frame, (center_x - qr_size, center_y - qr_size), 
                 (center_x - qr_size + corner_size, center_y - qr_size), corner_color, line_thickness)
        cv2.line(rotated_frame, (center_x - qr_size, center_y - qr_size), 
                 (center_x - qr_size, center_y - qr_size + corner_size), corner_color, line_thickness)
                 
        cv2.line(rotated_frame, (center_x + qr_size, center_y - qr_size), 
                 (center_x + qr_size - corner_size, center_y - qr_size), corner_color, line_thickness)
        cv2.line(rotated_frame, (center_x + qr_size, center_y - qr_size), 
                 (center_x + qr_size, center_y - qr_size + corner_size), corner_color, line_thickness)
                 
        cv2.line(rotated_frame, (center_x - qr_size, center_y + qr_size), 
                 (center_x - qr_size + corner_size, center_y + qr_size), corner_color, line_thickness)
        cv2.line(rotated_frame, (center_x - qr_size, center_y + qr_size), 
                 (center_x - qr_size, center_y + qr_size - corner_size), corner_color, line_thickness)
                 
        cv2.line(rotated_frame, (center_x + qr_size, center_y + qr_size), 
                 (center_x + qr_size - corner_size, center_y + qr_size), corner_color, line_thickness)
        cv2.line(rotated_frame, (center_x + qr_size, center_y + qr_size), 
                 (center_x + qr_size, center_y + qr_size - corner_size), corner_color, line_thickness)

        results = decode(rotated_frame)

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

        new_texture = Texture.create(size=(h, w))
        new_texture.blit_buffer(rotated_frame.tobytes(), colorfmt='rgba', bufferfmt='ubyte')
        self.image_display.texture = new_texture

    def flash_green_and_transition(self, passenger_name, flight_number, home, destination, dep_time, terminal, gate):
        self.qr_scanned = True

        flash = Widget(size_hint=(1, 1))
        flash.opacity = 1
        with flash.canvas:
            Color(0, 1, 0, 1)
            flash_rect = Rectangle(pos=self.pos, size=self.size)
        flash.bind(pos=lambda inst, val: setattr(flash_rect, 'pos', val),
                   size=lambda inst, val: setattr(flash_rect, 'size', val))
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
        
        # Label to display ticket details
        self.details_label = Label(
            text="",
            markup=True,  # Enable markup to process [b][/b] tags
            font_size=50,
            color=THEME_COLORS['text'],
            halign='center',
            valign='middle',
            pos_hint={'center_x': 0.5, 'center_y': 0.65}
        )
        self.main_layout.add_widget(self.details_label)
        
        # Create Yes/No button layout
        button_layout = BoxLayout(
            orientation='horizontal',
            size_hint=(0.5, 0.1),
            pos_hint={'center_x': 0.5, 'y': 0.3},
            spacing=20
        )
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
        # If yes, go back to the original FaceScreen (with the eyes)
        app = App.get_running_app()
        app.sm.transition = CardTransition(mode='pop')
        app.sm.current = "face"

    def on_no(self, instance):
        # If no, go back to the MenuScreen (or change this behavior as needed)
        self.qr_scanned = False
        app = App.get_running_app()
        app.sm.transition = CardTransition(mode='pop')
        app.sm.current = "menu"


# ------------------ Main Application ------------------
class SmoothOperatorApp(App):
    def build(self):
        self.title = "SmoothOperator"
        
        self.sm = ScreenManager(transition=FadeTransition(duration=0.5))
        
        face_screen = Screen(name="face")
        menu_screen = Screen(name="menu")
        manual_screen = ManualControlScreen(name="manual")
        # Pass the switch_to_postscan callback when creating the QR screen:
        qr_screen = QRScreen(switch_to_postscan=self.switch_to_postscan, name="qr")
        postscan_screen = PostScanScreen(name="postscan")

        face_widget = FaceScreen(self.switch_to_menu)
        face_screen.add_widget(face_widget)

        menu_widget = MenuScreen(self.switch_to_manual, self.switch_to_qr)
        menu_screen.add_widget(menu_widget)

        self.sm.add_widget(face_screen)
        self.sm.add_widget(menu_screen)
        self.sm.add_widget(manual_screen)
        self.sm.add_widget(qr_screen)
        self.sm.add_widget(postscan_screen)

        self.sm.current = "face"
        
        return self.sm

    def switch_to_menu(self):
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "menu"

    def switch_to_manual(self):
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "manual"

    def switch_to_qr(self):
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "qr"
        
    def switch_to_postscan(self, message=None):
        if message:
            postscan_screen = self.sm.get_screen("postscan")
            postscan_screen.update_postscan_message(message)
        self.sm.transition = CardTransition(mode='pop')
        self.sm.current = "postscan"

if __name__ == '__main__':
    SmoothOperatorApp().run()