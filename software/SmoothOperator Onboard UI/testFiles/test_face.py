# face_screen_test_proper.py

import random
import pygame
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.graphics import Ellipse, Color, Rectangle, Line
from kivy.animation import Animation
from kivy.properties import NumericProperty
from kivy.utils import get_color_from_hex

# Define theme colors
THEME_COLORS = {
    'primary': get_color_from_hex('#03A9F4'),
    'background': get_color_from_hex('#FFFFFF'),
    'accent': get_color_from_hex('#FFAC63'),
    'text': get_color_from_hex('#212121'),
    'success': get_color_from_hex('#4CAF50'),
    'error': get_color_from_hex('#F44336'),
}

# Set window size and background
Window.size = (1920, 1080)
Window.clearcolor = THEME_COLORS['background']

# Initialize pygame mixer for audio playback
pygame.mixer.init()

class MouthWidget(Widget):
    mouth_open = NumericProperty(0.1)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas:
            Color(*THEME_COLORS['primary'])
            self.smile_line = Line(bezier=[], width=4.0)
        self.bind(pos=self.update_mouth, size=self.update_mouth, mouth_open=self.update_mouth)
        
    def update_mouth(self, *args):
        width, height = self.width, self.height
        bottom_y = self.y
        smile_width = 0.9
        left_x = self.x + width * (1 - smile_width) / 2
        right_x = self.x + width * (1 + smile_width) / 2
        smile_height = height * self.mouth_open * 10
        points = [
            left_x, bottom_y,
            left_x + width / 4, bottom_y - smile_height,
            right_x - width / 4, bottom_y - smile_height,
            right_x, bottom_y
        ]
        self.smile_line.bezier = points

class FaceScreen(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.blinking = False
        self.eye_size = (Window.width * 0.25, Window.height * 0.5)
        self.eye_spacing = Window.width * 0.5
        
        # Create background (pre-rendered)
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
        self.bind(pos=self.update_positions, size=self.update_positions)
        
        # Create mouth widget and add it to the widget tree
        self.mouth_widget = MouthWidget(size_hint=(None, None), size=(self.eye_size[0] * 0.8, 30))
        self.add_widget(self.mouth_widget)
        self.bind(pos=self.update_mouth_position, size=self.update_mouth_position)
        
        # Schedule the blink event and store its reference
        self._blink_event = Clock.schedule_interval(self.blink, 5)  # every 5 seconds
        
        # Schedule audio playback after a delay (for testing)
        Clock.schedule_once(lambda dt: self.play_audio("audio/Hi_Im_SmoothOperator.mp3"), 3)
        
        # We'll store the audio-check event so we can cancel it later.
        self._audio_event = None

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_positions(self, *args):
        center_x, center_y = self.center
        left_x = center_x - self.eye_spacing / 2 - self.eye_size[0] / 2
        right_x = center_x + self.eye_spacing / 2 - self.eye_size[0] / 2
        y = center_y - self.eye_size[1] / 2
        self.left_eye.pos = (left_x, y)
        self.right_eye.pos = (right_x, y)

    def update_mouth_position(self, *args):
        self.mouth_widget.pos = (self.center_x - self.mouth_widget.width / 2,
                                 self.center_y - self.eye_size[1] / 2 - 20)

    def blink(self, dt):
        if not self.blinking:
            # Simulate a blink by reducing eye height.
            self.left_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.right_eye.size = (self.eye_size[0], self.eye_size[1] * 0.1)
            self.blinking = True
            Clock.schedule_once(self.unblink, 0.2)

    def unblink(self, dt):
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.blinking = False

    def play_audio(self, file_path):
        print("Playing audio with pygame...")
        try:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            self.start_mouth_animation()
            self._audio_event = Clock.schedule_interval(self.check_audio_finished, 0.1)
        except Exception as e:
            print("Audio playback error:", e)

    def check_audio_finished(self, dt):
        if not pygame.mixer.music.get_busy():
            self.stop_mouth_animation()
            if self._audio_event:
                self._audio_event.cancel()
                self._audio_event = None
            return False  # Unschedule this event

    def start_mouth_animation(self):
        self.mouth_anim = Animation(mouth_open=0.6, duration=0.3) + Animation(mouth_open=0.1, duration=0.3)
        self.mouth_anim.repeat = True
        self.mouth_anim.start(self.mouth_widget)

    def stop_mouth_animation(self):
        if hasattr(self, 'mouth_anim'):
            self.mouth_anim.cancel(self.mouth_widget)
        self.mouth_widget.mouth_open = 0.1

    def on_enter(self):
        # Ensure scheduling only once when the screen is entered.
        if not self._blink_event:
            self._blink_event = Clock.schedule_interval(self.blink, 5)

    def on_leave(self):
        # Cancel scheduled clock events when leaving.
        if self._blink_event:
            self._blink_event.cancel()
            self._blink_event = None
        if self._audio_event:
            self._audio_event.cancel()
            self._audio_event = None

class FaceApp(App):
    def build(self):
        return FaceScreen()

if __name__ == '__main__':
    FaceApp().run()

