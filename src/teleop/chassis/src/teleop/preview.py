"""Read-only/offline target preview. This module has no robot transport."""
from pathlib import Path as _ProjectPath
import sys as _project_sys
_project_sys.path.insert(0,str(_ProjectPath(__file__).resolve().parents[1]/"common"))
from project_paths import configure_imports, output_directory
configure_imports()
import argparse
import array
from dataclasses import asdict
from datetime import datetime, timedelta
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import struct
import sys
import time
import uuid
from zoneinfo import ZoneInfo
from controller import DpadController, InputFrame, SafetyState

ROOT = Path(__file__).resolve().parents[2]
TZ = ZoneInfo("Asia/Shanghai")


def apply_js_event(axes, buttons, raw):
    if len(raw) != 8:
        raise ValueError("Joystick event must contain exactly 8 bytes")
    _, value, kind, index = struct.unpack("IhBB", raw)
    kind &= 0x7f
    if kind == 1:
        if index >= len(buttons) or value not in (0, 1):
            raise ValueError("Invalid joystick button event")
        buttons[index] = value
    elif kind == 2:
        if index >= len(axes):
            raise ValueError("Invalid joystick axis index")
        axes[index] = max(-1.0, min(1.0, value / 32767.0))
    else:
        raise ValueError("Unknown joystick event type")


class JoystickEventStream:
    """Preserve every edge and kernel millisecond interval, including queued taps."""
    def __init__(self):
        self.axes = [0.0] * 8
        self.buttons = [0] * 13
        self._axes_seen = set()
        self._buttons_seen = set()
        self._last_stamp = None
        self._last_time = None
        self._real_event_seen = False

    def feed(self, raw, observed_now):
        if len(raw) != 8:
            raise ValueError("Joystick event must contain exactly 8 bytes")
        stamp, _, kind, index = struct.unpack("IhBB", raw)
        if kind & 0x80:
            if self._last_stamp is not None:
                raise ValueError("Unexpected device reinitialization")
            apply_js_event(self.axes, self.buttons, raw)
            (self._buttons_seen if kind & 1 else self._axes_seen).add(index)
            if len(self._axes_seen) == 8 and len(self._buttons_seen) == 13:
                self._last_stamp, self._last_time = stamp, observed_now
                return [(observed_now, InputFrame(tuple(self.axes), tuple(self.buttons)))]
            return []
        if self._last_stamp is None:
            raise ValueError("Initial joystick baseline is incomplete")
        delta_ms = (stamp - self._last_stamp) & 0xffffffff
        if delta_ms > 0x7fffffff:
            if not self._real_event_seen:
                return []  # An event queued before the initial state snapshot.
            raise ValueError("Joystick event clock moved backwards")
        apply_js_event(self.axes, self.buttons, raw)
        self._last_time += delta_ms / 1000.0
        self._last_stamp = stamp
        self._real_event_seen = True
        return [(self._last_time, InputFrame(tuple(self.axes), tuple(self.buttons)))]


class Joystick:
    """Independent read-only descriptor; does not grab or change the device."""
    def __init__(self, device):
        self.device = device
        self.fd = None
        self.events = JoystickEventStream()

    def __enter__(self):
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        try:
            if not stat.S_ISCHR(os.fstat(self.fd).st_mode):
                raise ValueError("Expected a Linux joystick character device")
            axes = array.array("B", [0])
            buttons = array.array("B", [0])
            fcntl.ioctl(self.fd, 0x80016a11, axes, True)
            fcntl.ioctl(self.fd, 0x80016a12, buttons, True)
            axmap = array.array("B", [0] * 64)
            keymap = array.array("H", [0] * 512)
            fcntl.ioctl(self.fd, 0x80406a32, axmap, True)
            fcntl.ioctl(self.fd, 0x84006a34, keymap, True)
            if axes[0] != 8 or buttons[0] != 13 or list(axmap[6:8]) != [16, 17] or keymap[4] != 310 or keymap[3] != 308:
                raise ValueError("Device layout differs from confirmed 8-axis/13-button DS4 mapping")
            return self
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise

    def poll(self, observed_now):
        for _ in range(512):
            try:
                raw = os.read(self.fd, 8)
            except BlockingIOError:
                break
            if len(raw) != 8:
                raise OSError("Joystick disconnected or short read")
            yield from self.events.feed(raw, observed_now)

    def __exit__(self, *_):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def open_project_log(source):
    if source not in ("demo", "joystick"):
        raise ValueError("Unknown log source")
    folder = output_directory("logs", ROOT) / "preview" / datetime.now(TZ).strftime("%Y-%m-%d")
    if folder.is_symlink() or not folder.resolve().is_relative_to((output_directory("logs", ROOT)).resolve()):
        raise ValueError("Log directory must remain inside this project")
    folder.mkdir(parents=True, exist_ok=True)
    directory = os.open(folder, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        name = f"{source}-{datetime.now(TZ):%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}.jsonl"
        fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                     0o600, dir_fd=directory)
        return folder / name, os.fdopen(fd, "w", encoding="utf-8")
    finally:
        os.close(directory)


def output(row, stream=None):
    stream = sys.stdout if stream is None else stream
    text = json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
    try:
        try:
            fd = stream.fileno()
        except (AttributeError, OSError, ValueError):
            stream.write(text)
            stream.flush()
        else:
            pending = memoryview(text.encode("utf-8"))
            while pending:
                pending = pending[os.write(fd, pending):]
    except (OSError, ValueError):
        return False
    return True


class LogWriteError(OSError):
    pass


def show_result(result, source, log, out=None):
    rows = [{"type": "press", **asdict(event), "mode": "offline_preview",
             "input_source": source, "sent_to_robot": False} for event in result.events]
    failure = None
    if log is not None:
        try:
            for row in rows:
                log.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
        except OSError as exc:
            failure = exc
    # Persist records before console output; a closed terminal cannot discard logs.
    output({"type": "target", **asdict(result.command), "mode": "offline_preview",
            "input_source": source, "sent_to_robot": False}, out)
    for row in rows:
        output(row, out)
    if failure is not None:
        raise LogWriteError(str(failure)) from failure


def main(argv=None):
    parser = argparse.ArgumentParser(description="离线十字键预览：只读输入、打印目标、记录时长；不会控制机器人。")
    parser.add_argument("--demo", action="store_true", help="使用合成输入，不读取手柄")
    parser.add_argument("--simulate-ready", action="store_true", help="仅模拟软件使能/手动/新鲜度条件；不启用任何真实服务")
    parser.add_argument("--device", default="/dev/input/js0", help="只读 Linux joystick 路径")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--no-log", action="store_true", help="仅打印，不保存日志")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and greater than zero")
    source = "demo" if args.demo else "joystick"
    controller = DpadController()
    safety = SafetyState(True, True, 0.0) if args.simulate_ready else SafetyState()
    log = None
    now = 0.0
    timestamp = datetime.now(TZ).isoformat()
    started = time.monotonic()
    wall_start = datetime.now(TZ)
    exit_code = 0
    try:
        path = None
        if not args.no_log:
            path, log = open_project_log(source)
        output({"type": "startup", "mode": "offline_preview", "input_source": source,
                "safety_source": "simulated_ready" if args.simulate_ready else "not_ready",
                "freshness_source": "assumed_for_offline_preview", "sent_to_robot": False,
                "log_path": str(path) if path else None})
        if args.demo:
            wall = datetime.now(TZ)
            samples = [(0,0,0,0),(.02,0,0,1),(1,0,-1,1),(3,0,0,1),
                       (4,0,1,1),(5,0,0,1),(6,-1,0,1),(8,0,0,1),
                       (9,1,0,1),(9.5,0,0,1)]
            for now, x, y, l1 in samples:
                axes = [0.0] * 8; axes[6:8] = [x, y]
                buttons = [0] * 13; buttons[4] = l1
                timestamp = (wall + timedelta(seconds=now)).isoformat()
                show_result(controller.update(InputFrame(tuple(axes), tuple(buttons)), safety, now, timestamp), source, log)
        else:
            with Joystick(args.device) as joystick:
                while time.monotonic() - started < args.seconds:
                    tick = time.monotonic()
                    for event_time, frame in joystick.poll(tick - started):
                        now = event_time
                        timestamp = (wall_start + timedelta(seconds=now)).isoformat()
                        show_result(controller.update(frame, safety, now, timestamp), source, log)
                    time.sleep(max(0.0, .02 - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        exit_code = 130
    except LogWriteError as exc:
        if log is not None:
            try:
                log.close()
            except OSError:
                pass
        log = None
        output({"type": "error", "error": "Log write failed: " + str(exc), "sent_to_robot": False})
        exit_code = 1
    except (OSError, ValueError) as exc:
        now = max(now, time.monotonic() - started)
        timestamp = datetime.now(TZ).isoformat()
        lost = InputFrame((0.0,) * 8, (0,) * 13, connected=False)
        show_result(controller.update(lost, safety, now, timestamp), source, log)
        output({"type": "error", "error": str(exc), "sent_to_robot": False})
        exit_code = 1
    finally:
        if not args.demo:
            now = max(now, time.monotonic() - started)
            timestamp = datetime.now(TZ).isoformat()
        try:
            show_result(controller.finish(now, timestamp), source, log)
        finally:
            if log is not None:
                log.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
