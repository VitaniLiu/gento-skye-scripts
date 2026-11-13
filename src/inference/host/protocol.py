"""Small length-prefixed JSON implementation of the inference wire protocol."""
import base64
from io import BytesIO
import json
import numpy as np
from PIL import Image
import socket
import struct
import uuid


def raise_server_error(reply):
    error = reply.get("error")
    if error is not None or reply.get("ok") is False:
        if isinstance(error, dict):
            error = error.get("message", error)
        raise RuntimeError(str(error if error is not None else "server rejected request"))


def encode_array(value):
    if isinstance(value, np.ndarray):
        value = np.ascontiguousarray(value)
        return {"__array__": True, "dtype": value.dtype.str,
                "shape": list(value.shape),
                "data": base64.b64encode(value.tobytes()).decode("ascii")}
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("unsupported wire value: " + type(value).__name__)


def decode_array(value):
    if value.get("__array__") is True:
        return np.frombuffer(base64.b64decode(value["data"]),
                             dtype=np.dtype(value["dtype"])).reshape(value["shape"]).copy()
    return value


def native_request(request):
    """Adapt the transport only; poses remain native base-relative flanges."""
    command = request.get("command", request.get("cmd"))
    if command == "info":
        return {"cmd": "info"}
    if command == "reset":
        return {"cmd": "reset", "request_id": request.get("request_id", uuid.uuid4().hex)}
    if command != "infer":
        return dict(request, cmd=command)
    obs = request["observation"]
    frames = obs["images"]["head_color"]
    images = [np.asarray(Image.open(BytesIO(base64.b64decode(frame))).convert("RGB"))
              for frame in frames]
    observation = dict(state=np.asarray(obs["state"], dtype=np.float32), head_rgb=images[-1])
    if len(images) > 1:
        observation.update(head_rgb_history=np.stack(images),
                           history_offsets=[-16, -12, -8, -4, 0],
                           history_is_pad=obs["image_padding_mask"])
    return dict(cmd="infer", task=request["task"],
                embodiment=request.get("embodiment", "skye"),
                state_frame="source", request_id=request["request_id"],
                session_id=request["session_id"], sample_ns=request["observation_timestamp_ns"],
                observation=observation)


class PolicyClient:
    def __init__(self, host, port, timeout, wire_protocol="json_length_v1"):
        self.wire_protocol = wire_protocol
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def request(self, command, **fields):
        return self.request_object({"command": command, **fields})

    def request_object(self, request):
        if self.wire_protocol == "egoav78_chunk_v1":
            request = native_request(request)
        payload = json.dumps(request, default=encode_array, allow_nan=False, separators=(",", ":")).encode()
        self.socket.sendall(struct.pack("!I", len(payload)) + payload)
        size = struct.unpack("!I", self._read(4))[0]
        if size > 128 * 1024 * 1024:
            raise RuntimeError("server response is too large")
        reply = json.loads(self._read(size), object_hook=decode_array)
        if not isinstance(reply, dict):
            raise RuntimeError("server response must be an object")
        raise_server_error(reply)
        return reply

    def _read(self, size):
        parts = []
        while size:
            block = self.socket.recv(size)
            if not block:
                raise ConnectionError("server closed the connection")
            parts.append(block)
            size -= len(block)
        return b"".join(parts)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
