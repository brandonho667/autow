from pydub import AudioSegment
from pydub.playback import play

sound = AudioSegment.from_mp3("beep-01a.mp3")
play(sound)
