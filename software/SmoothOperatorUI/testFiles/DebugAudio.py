from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
import pygame

# Initialize pygame mixer (do this before loading or playing any audio)
pygame.mixer.init()

# Path to the audio file (ensure the file exists and is in a format supported by pygame)
AUDIO_FILE = "PythonUI/audio/Hi_Im_SmoothOperator.mp3"

class DebugAudioApp(App):
    def build(self):
        layout = BoxLayout(orientation="vertical", padding=50, spacing=20)
        play_button = Button(text="Play Audio", font_size=40)
        stop_button = Button(text="Stop Audio", font_size=40)
        
        play_button.bind(on_press=lambda instance: self.play_audio(AUDIO_FILE))
        stop_button.bind(on_press=lambda instance: self.stop_audio())
        
        layout.add_widget(play_button)
        layout.add_widget(stop_button)
        return layout

    def play_audio(self, file_path):
        print("Playing audio with pygame...")
        try:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
        except Exception as e:
            print("Error playing audio:", e)

    def stop_audio(self):
        print("Stopping audio with pygame...")
        pygame.mixer.music.stop()

if __name__ == '__main__':
    DebugAudioApp().run()
