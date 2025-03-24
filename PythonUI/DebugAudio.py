from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.core.audio import SoundLoader

# Path to the audio file (ensure this file exists in the correct relative location)
AUDIO_FILE = "audio/Hi_Im_SmoothOperator.mp3"

class DebugAudioApp(App):
    def build(self):
        self.sound = None  # This will hold our loaded sound
        layout = BoxLayout(orientation="vertical", padding=50, spacing=20)
        
        play_button = Button(text="Play Audio", font_size=40)
        stop_button = Button(text="Stop Audio", font_size=40)
        
        # Use lambda so that the function is called only on button press
        play_button.bind(on_press=lambda instance: self.play_audio(AUDIO_FILE))
        stop_button.bind(on_press=lambda instance: self.stop_audio())
        
        layout.add_widget(play_button)
        layout.add_widget(stop_button)
        return layout

    def play_audio(self, file_path):
        # Stop the previous sound if it's already playing
        print("Playing Audio")
        if self.sound:
            self.sound.stop()
        self.sound = SoundLoader.load(file_path)
        if self.sound:
            print(f"Audio file '{file_path}' loaded successfully. Playing audio...")
            self.sound.play()
        else:
            print("Unable to load audio file:", file_path)

    def stop_audio(self):
        if self.sound:
            print("Stopping audio...")
            self.sound.stop()
        else:
            print("No audio is currently playing.")

if __name__ == '__main__':
    DebugAudioApp().run()
