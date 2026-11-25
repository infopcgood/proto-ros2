"""Explicit LED order and anatomical side mapping, independent of ROS/driver."""
import json
from pathlib import Path
import numpy as np


class AtlasMap:
    def __init__(self, configuration):
        try:
            self._load(configuration)
        except (AttributeError, KeyError, TypeError, OverflowError) as exc:
            raise ValueError('Malformed panel-map configuration') from exc

    def _load(self, configuration):
        if configuration.get('schema') != 1:
            raise ValueError('Unsupported panel-map schema')

        def integer(name):
            value = configuration[name]
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(name + ' must be a positive integer')
            return value

        self.width = integer('side_width')
        self.height = integer('side_height')
        self.leds_per_strip = integer('leds_per_strip')
        self.num_strips = integer('num_strips')
        pixels = configuration['pixels']
        count = self.leds_per_strip * self.num_strips
        if len(pixels) != count:
            raise ValueError('Map must have exactly one entry per driver LED slot')
        self.x = np.zeros(count, dtype=np.intp)
        self.y = np.zeros(count, dtype=np.intp)
        self.valid = np.zeros(count, dtype=bool)
        for i, entry in enumerate(pixels):
            if entry is None:
                continue
            if not isinstance(entry, (list, tuple)) or len(entry) != 3:
                raise ValueError('LED entries must be [left|right,x,y] or null')
            side, x, y = entry
            if (side not in ('left', 'right') or isinstance(x, bool) or isinstance(y, bool)
                    or not isinstance(x, int) or not isinstance(y, int)
                    or not 0 <= x < self.width or not 0 <= y < self.height):
                raise ValueError('Invalid side/pixel coordinate in panel map')
            self.x[i] = x + (self.width if side == 'right' else 0)
            self.y[i] = y
            self.valid[i] = True

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).expanduser().read_text()))

    def rgb_bytes(self, data, width, height, flip_second_screen=False):
        if width != self.width * 2 or height != self.height:
            raise ValueError('Frame dimensions do not match the two-side atlas map')
        if flip_second_screen:
            raise ValueError('Side-specific atlas must not request a second mirror')
        if isinstance(data, list):
            values = np.asarray(data)
            if (values.ndim != 1 or values.dtype.kind not in 'ui'
                    or np.any(values < 0) or np.any(values > 255)):
                raise ValueError('BGR list must contain uint8 values')
            frame = values.astype(np.uint8)
        else:
            buffer = memoryview(data)
            if buffer.ndim != 1 or buffer.itemsize != 1 or not buffer.c_contiguous:
                raise ValueError('BGR payload must be a contiguous byte buffer')
            frame = np.frombuffer(buffer, dtype=np.uint8)
        if frame.size != width * height * 3:
            raise ValueError('BGR payload length mismatch')
        frame = frame.reshape(height, width, 3)
        pixels = frame[self.y, self.x, ::-1].copy()
        pixels[~self.valid] = 0
        return pixels.tobytes()
