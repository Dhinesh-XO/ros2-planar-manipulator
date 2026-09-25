"""Record only this Qt window; encoding runs outside the GUI event loop."""

from pathlib import Path
import queue
import subprocess
import threading
import time

from PyQt5 import QtGui


class WindowRecorder:
    def __init__(self, path, width, height, fps=10):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.frames = queue.Queue(maxsize=8)
        self.error = None
        self.dropped = 0
        self.fps = fps
        self.encoded_frames = 0
        self.max_capture_seconds = 0.0
        self.process = subprocess.Popen([
            'ffmpeg', '-n', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgba',
            '-video_size', f'{width}x{height}', '-framerate', str(fps), '-i', 'pipe:0',
            '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20',
            '-pix_fmt', 'yuv420p', str(path)], stdin=subprocess.PIPE)
        self.thread = threading.Thread(target=self.encode, name='video-encoder')
        self.thread.start()

    def capture(self, window):
        started = time.monotonic()
        image = window.grab().toImage().convertToFormat(QtGui.QImage.Format_RGBA8888)
        self.max_capture_seconds = max(self.max_capture_seconds, time.monotonic()-started)
        try:
            self.frames.put_nowait((time.monotonic(), image.bits().asstring(image.byteCount())))
        except queue.Full:
            self.dropped += 1

    def encode(self):
        origin, previous = None, None
        try:
            while True:
                frame = self.frames.get()
                if frame is None:
                    break
                timestamp, pixels = frame
                if origin is None:
                    origin = timestamp
                target_index = round((timestamp-origin)*self.fps)
                # Preserve elapsed wall time under load. Missing captures show
                # a held frame, never silently accelerate the demonstration.
                while self.encoded_frames < target_index:
                    self.process.stdin.write(previous if previous is not None else pixels)
                    self.encoded_frames += 1
                if self.encoded_frames == target_index:
                    self.process.stdin.write(pixels)
                    self.encoded_frames += 1
                previous = pixels
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
        print(f'Recording: {self.encoded_frames} frames; queue drops {self.dropped}; '
              f'maximum window capture {self.max_capture_seconds:.3f}s', flush=True)
