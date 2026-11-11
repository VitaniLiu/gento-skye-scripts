"""Task-specific RGB preprocessing and causal five-frame history for SKYE."""
import base64
from bisect import bisect_right
from collections import deque
from io import BytesIO
import threading
import numpy as np
from PIL import Image


def prepare_rgb(array, spec):
    image = Image.fromarray(np.asarray(array, dtype=np.uint8)).convert("RGB")
    if list(image.size) != spec["input_size"]:
        raise ValueError(f"expected image {spec['input_size']}, received {image.size}")
    box = spec["crop_xyxy"]
    if not (0 <= box[0] < box[2] <= image.width and 0 <= box[1] < box[3] <= image.height):
        raise ValueError("crop outside input image")
    return image.crop(tuple(box)).resize(tuple(spec["output_size"]), Image.Resampling.LANCZOS)


def encode_rgb(image, encoding="PNG"):
    stream = BytesIO()
    image.save(stream, format=encoding)
    return base64.b64encode(stream.getvalue()).decode("ascii")


class ImageHistory:
    def __init__(self, spec, offsets):
        self.spec = spec
        self.offsets = offsets
        self.frames = deque(maxlen=256)
        self.lock = threading.Lock()

    def append(self, timestamp_ns, rgb):
        payload = encode_rgb(prepare_rgb(rgb, self.spec), self.spec["encoding"])
        with self.lock:
            if self.frames and timestamp_ns <= self.frames[-1][0]:
                return
            self.frames.append((int(timestamp_ns), payload))

    def latest_before(self, cutoff_ns):
        with self.lock:
            frame = next((f for f in reversed(self.frames) if f[0] <= cutoff_ns), None)
        if frame is None:
            raise RuntimeError("no head image at or before observation time")
        return frame[0]

    def sample(self, t0_ns):
        with self.lock:
            frames = [f for f in self.frames if f[0] <= t0_ns]
        if not frames:
            raise RuntimeError("no head image at or before observation time")
        stamps = [f[0] for f in frames]
        targets = [t0_ns + round(t * 1e9) for t in self.offsets]
        slots = [bisect_right(stamps, t) - 1 for t in targets]
        return {"head_color": [frames[max(0, i)][1] for i in slots],
                "image_timestamps_ns": [frames[max(0, i)][0] for i in slots],
                "image_target_timestamps_ns": targets,
                "image_padding_mask": [i < 0 for i in slots],
                "image_encoding": self.spec["encoding"].lower(),
                "image_size": self.spec["output_size"],
                "crop_xyxy": self.spec["crop_xyxy"]}
