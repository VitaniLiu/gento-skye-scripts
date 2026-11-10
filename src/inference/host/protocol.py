"""Small length-prefixed JSON implementation of the inference wire protocol."""
import json
import socket
import struct


class PolicyClient:
    def __init__(self, host, port, timeout):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def request(self, command, **fields):
        return self.request_object({"command": command, **fields})

    def request_object(self, request):
        payload = json.dumps(request, allow_nan=False, separators=(",", ":")).encode()
        self.socket.sendall(struct.pack("!I", len(payload)) + payload)
        size = struct.unpack("!I", self._read(4))[0]
        if size > 128 * 1024 * 1024:
            raise RuntimeError("server response is too large")
        reply = json.loads(self._read(size))
        if not isinstance(reply, dict):
            raise RuntimeError("server response must be an object")
        if reply.get("ok") is False:
            raise RuntimeError(str(reply.get("error", "server rejected request")))
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
