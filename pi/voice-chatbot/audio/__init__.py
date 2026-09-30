"""Audio layer — raw PCM capture and playback via PyAudio."""
from audio.mic import Microphone
from audio.speaker import Speaker

__all__ = ["Microphone", "Speaker"]
