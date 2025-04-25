from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Ellipse, Color
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button

class FaceScreen(Widget):
    def __init__(self, switch_screen_callback, **kwargs):
        super().__init__(**kwargs)
        self.switch_screen_callback = switch_screen_callback
        self.blinking = False
        self.eye_size = (80, 140)
        self.left_eye_pos = (self.center_x - 120, self.center_y)
        self.right_eye_pos = (self.center_x + 40, self.center_y)
        
        with self.canvas:
            Color(0, 191/255, 255/255)
            self.left_eye = Ellipse(pos=self.left_eye_pos, size=self.eye_size)
            self.right_eye = Ellipse(pos=self.right_eye_pos, size=self.eye_size)
        
        Clock.schedule_interval(self.blink, 3)
        self.bind(pos=self.update_positions, size=self.update_positions)
    
    def update_positions(self, *args):
        self.left_eye.pos = (self.center_x - 120, self.center_y)
        self.right_eye.pos = (self.center_x + 40, self.center_y)
    
    def blink(self, dt):
        if not self.blinking:
            self.left_eye.size = (80, 20)
            self.right_eye.size = (80, 20)
            self.blinking = True
            Clock.schedule_once(self.unblink, 0.2)
    
    def unblink(self, dt):
        self.left_eye.size = self.eye_size
        self.right_eye.size = self.eye_size
        self.blinking = False
    
    def on_touch_down(self, touch):
        self.switch_screen_callback()

class NextScreen(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', **kwargs)
        self.add_widget(Button(text="Welcome to SmoothOperator™", font_size=24))
        self.add_widget(Button(text="Proceed", font_size=20))

class SmoothOperatorApp(App):
    def build(self):
        self.face_screen = FaceScreen(self.switch_to_next_screen)
        self.next_screen = NextScreen()
        self.root_widget = self.face_screen
        return self.root_widget
    
    def switch_to_next_screen(self):
        self.root_widget = self.next_screen
        self.root.clear_widgets()
        self.root.add_widget(self.next_screen)

if __name__ == '__main__':
    SmoothOperatorApp().run()

