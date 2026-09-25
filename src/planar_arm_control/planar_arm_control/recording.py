"""Record only this Qt window; encoding runs outside the GUI event loop."""

from pathlib import Path
import queue
import subprocess
import threading

from PyQt5 import QtGui


class WindowRecorder:
    def __init__(self, path, width, height, fps=10):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.frames = queue.Queue(maxsize=8)
        self.error = None
        self.dropped = 0
        self.process = subprocess.Popen([
            'ffmpeg', '-n', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgba',
            '-video_size', f'{width}x{height}', '-framerate', str(fps), '-i', 'pipe:0',
            '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20',
            '-pix_fmt', 'yuv420p', str(path)], stdin=subprocess.PIPE)
        self.thread = threading.Thread(target=self.encode, name='video-encoder')
        self.thread.start()

    def capture(self, window):
        image = window.grab().toImage().convertToFormat(QtGui.QImage.Format_RGBA8888)
        try:
            self.frames.put_nowait(image.bits().asstring(image.byteCount()))
        except queue.Full:
            self.dropped += 1

    def encode(self):
        try:
            while True:
                frame = self.frames.get()
                if frame is None:
                    break
                self.process.stdin.write(frame)
        except (BrokenPipeError, OSError) as error:
            self.error = str(error)
        finally:
            self.process.stdin.close()
            self.process.wait(timeout=15)

    def close(self):
        if self.thread.is_alive():
            self.frames.put(None, timeout=5)
        self.thread.join(timeout=20)
        if self.error or self.process.returncode != 0 or self.thread.is_alive():
            raise RuntimeError(f'Video encoding failed: {self.error}')
