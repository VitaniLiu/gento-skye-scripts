"""Nonblocking JSON pipes and independent USB report liveness."""
import json
import math
import os

class ChannelError(RuntimeError):
    pass


def _invalid_constant(value):
    raise ValueError("Non-finite JSON constant: " + value)


class JsonChannel:
    LIMIT = 4096
    def __init__(self, read_fd, write_fd):
        self.read_fd, self.write_fd = read_fd, write_fd
        self.buffer = b""
        self.eof = False
        if read_fd is not None: os.set_blocking(read_fd,False)
        if write_fd is not None: os.set_blocking(write_fd,False)

    def send(self, message):
        try:
            raw = (json.dumps(message,separators=(",",":"),allow_nan=False)+"\n").encode()
            if len(raw)>self.LIMIT or self.write_fd is None:
                raise ChannelError("Message exceeds pipe limit or channel is read-only")
            if os.write(self.write_fd,raw)!=len(raw):
                raise ChannelError("Partial pipe write; refusing buffered replay")
        except (OSError,ValueError,TypeError) as exc:
            raise ChannelError("Pipe write failed: "+str(exc)) from exc

    def receive(self):
        if self.eof:return []
        messages=[]
        for _ in range(32):
            try:chunk=os.read(self.read_fd,self.LIMIT)
            except BlockingIOError:break
            except OSError as exc:raise ChannelError(str(exc)) from exc
            if not chunk:
                self.eof=True
                if self.buffer:raise ChannelError("EOF inside a JSON message")
                break
            self.buffer+=chunk
            while b"\n" in self.buffer:
                raw,self.buffer=self.buffer.split(b"\n",1)
                if len(raw)>self.LIMIT:raise ChannelError("Oversized input message")
                try:
                    value=json.loads(raw,parse_constant=_invalid_constant)
                    if not isinstance(value,dict):raise ValueError("Object required")
                except (ValueError,UnicodeError) as exc:raise ChannelError("Invalid JSON: "+str(exc)) from exc
                messages.append(value)
                if len(messages)>128:raise ChannelError("Input flood")
            if len(self.buffer)>self.LIMIT:raise ChannelError("Oversized unfinished message")
        return messages


class HidTracker:
    def __init__(self):
        self.last_packet=None
        self.last_change=None
    def observe(self, packet, now):
        if not isinstance(packet,bytes) or len(packet)!=64 or packet[0]!=1:
            raise ValueError("Expected a 64-byte DS4 USB report with ID 1")
        if packet!=self.last_packet:
            self.last_packet=packet;self.last_change=now
    def age(self, now):
        if self.last_change is None or now<self.last_change:return float("inf")
        return now-self.last_change
