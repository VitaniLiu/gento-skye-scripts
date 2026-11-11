#!/usr/bin/env python3
"""Host-side Gento Skye inference loop."""
import argparse
import base64
from io import BytesIO
from fractions import Fraction
import json
import os
from pathlib import Path
import queue
import re
import select
import socket
import subprocess
import sys
import threading
import time
import uuid

import av
import numpy as np
from PIL import Image

from kinematics import JOINTS, Kinematics
from protocol import PolicyClient
from images import ImageHistory
from startup import StartupCancelled, align_startup, check_stop


ROOT = Path(__file__).resolve().parents[3]


def load_configuration(path, model):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", model):
        raise ValueError("invalid model name")
    config = json.loads(path.read_text())
    deployment_path = path.parent / f"deployed_{model}.json"
    deployment = json.loads(deployment_path.read_text())
    if deployment.get("wire_protocol") not in ("json_length_v1", "egoav78_chunk_v1"):
        raise ValueError("unsupported wire protocol")
    if deployment.get("transport", "ssh") not in ("ssh", "tcp"):
        raise ValueError("unknown server transport")
    instruction_mode = deployment.get("instruction_mode", "request")
    if instruction_mode not in ("request", "server"):
        raise ValueError("unknown instruction mode")
    tasks_path = deployment_path.parent / deployment["tasks_file"]
    settings = config["inference"]
    settings["model"] = deployment["model"]
    settings["image_history_offsets_s"] = deployment.get("image_history_offsets_s", settings["image_history_offsets_s"])
    settings["instruction_mode"] = instruction_mode
    settings["tasks"] = json.loads(tasks_path.read_text())["tasks"]
    config["server"] = deployment
    return config


def jpeg_base64(array):
    stream = BytesIO()
    Image.fromarray(np.asarray(array, dtype=np.uint8)).save(stream, format="JPEG", quality=90)
    return base64.b64encode(stream.getvalue()).decode("ascii")


class CameraDecoder:
    def __init__(self):
        self.decoder = av.CodecContext.create("h264", "r")

    def decode(self, timestamp_ns, encoded):
        result = None
        packet = av.Packet(base64.b64decode(encoded))
        packet.pts = timestamp_ns
        packet.time_base = Fraction(1, 1_000_000_000)
        try:
            frames = self.decoder.decode(packet)
        except av.error.InvalidDataError:
            return None
        for frame in frames:
            array = frame.to_ndarray(format="rgb24")
            stamp = round(frame.pts * frame.time_base * 1_000_000_000) if frame.pts is not None else timestamp_ns
            result = (stamp, array)
        return result


class RobotClient:
    def __init__(self, config, decode_camera=True, task=None):
        self.history = None
        if decode_camera:
            settings = config["inference"]
            self.history = ImageHistory(settings["tasks"][task]["image"], settings["image_history_offsets_s"])
        robot = config["robot"]
        remote_root = robot["remote_root"]
        shell = ("source /etc/apex/apex_ros_env.sh >/dev/null 2>&1 && exec python3 -u -B "
                 + remote_root + "/src/inference/robot/runtime.py --config "
                 + "-")
        self.process = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", robot["ssh"], shell],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=sys.stderr, text=True, bufsize=1)
        self.messages = queue.Queue()
        self.process.stdin.write(json.dumps(config, allow_nan=False) + "\n")
        self.process.stdin.flush()
        self.decode_camera = decode_camera
        self.image = None
        self.image_lock = threading.Lock()
        self.image_after_ns = 0
        self.decoder = CameraDecoder()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                message = json.loads(line)
                if message.get("type") == "camera":
                    if not self.decode_camera:
                        continue
                    image = self.decoder.decode(message["timestamp_ns"], message["data"])
                    if image is not None:
                        with self.image_lock:
                            if image[0] >= self.image_after_ns:
                                self.history.append(*image)
                else:
                    self.messages.put(message)
            except Exception as exc:
                self.messages.put({"type": "error", "error": "robot stream: " + str(exc)})
        self.messages.put({"type": "error", "error": "robot endpoint closed"})

    def send(self, command, **fields):
        self.process.stdin.write(json.dumps({"command": command, **fields}, allow_nan=False) + "\n")
        self.process.stdin.flush()

    def wait(self, expected, timeout=120):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("robot endpoint timeout waiting for " + expected)
            message = self.messages.get(timeout=remaining)
            if message.get("type") == "stage":
                print("robot:", message["stage"], flush=True)
                continue
            if message.get("type") == "error":
                raise RuntimeError(message["error"])
            if message.get("type") == expected:
                return message

    def reset_image_history(self, after_ns):
        # Gate late decoded packets as well as frames already in the buffer.
        with self.image_lock:
            self.image_after_ns = int(after_ns) + 1
            with self.history.lock:
                self.history.frames.clear()

    def image_anchor(self, cutoff_ns):
        return self.history.latest_before(cutoff_ns)

    def image_history(self, timestamp_ns):
        return self.history.sample(timestamp_ns)

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.close()
            except OSError:
                pass
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=2)


class Tunnel:
    def __init__(self, config):
        server = config["server"]
        self.process = subprocess.Popen([
            "ssh", "-N", "-o", "ExitOnForwardFailure=yes", "-o", "BatchMode=yes",
            "-L", f'{server["local_port"]}:{server["server_host"]}:{server["server_port"]}',
            server["ssh_host"],
        ])
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("SSH inference tunnel exited")
            try:
                with socket.create_connection(("127.0.0.1", server["local_port"]), timeout=.2):
                    return
            except OSError:
                time.sleep(.1)
        raise TimeoutError("SSH inference tunnel did not become ready")

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=2)


def server_address(config):
    server = config["server"]
    transport = server.get("transport", "ssh")
    if transport == "tcp":
        return server["server_host"], server["server_port"], None
    if transport == "ssh":
        if "parallel_lanes" in server:
            if server["parallel_lanes"] != 64:
                raise ValueError("parallel transport requires64 lanes")
            from parallel_transport import ParallelTunnel
            tunnel = ParallelTunnel(server)
        else:
            tunnel = Tunnel(config)
        return "127.0.0.1", server["local_port"], tunnel
    raise ValueError("unknown server transport: " + str(transport))


def q_listener(event, on_stop=None):
    if not sys.stdin.isatty():
        return
    import termios
    import tty
    fd, previous = sys.stdin.fileno(), termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(fd)
        while not event.is_set():
            if select.select([fd], [], [], .1)[0] and os.read(fd, 1).lower() == b"q":
                event.set()
                if on_stop is not None:
                    on_stop()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def check_info(info, config, task, instruction, steps):
    from protocol import raise_server_error
    raise_server_error(info)
    expected = config["inference"]
    checks = (("action_dim", expected["action_dim"]),
              ("action_horizon", expected["action_horizon"]),
              ("action_dt_s", expected["action_dt_s"]))
    for key, value in checks:
        if key in info and info[key] != value:
            raise RuntimeError(f"server {key}={info[key]!r}, expected {value!r}")
    server_task = expected["tasks"][task].get("server_task", task)
    if "tasks" in info and server_task not in info["tasks"]:
        raise RuntimeError("server does not provide task " + server_task)
    if expected["instruction_mode"] == "server":
        task_info = info.get("tasks", {}).get(server_task, {})
        if not isinstance(task_info, dict) or task_info.get("prompt") != instruction:
            raise RuntimeError("server task instruction does not match client configuration")
    if steps > expected["action_horizon"]:
        raise ValueError("steps exceeds action horizon")


def task_spec(settings, task):
    record = settings["tasks"][task]
    mask = [True] * settings["action_dim"]
    for index in record.get("masked_indices", []):
        mask[index] = False
    for start, end in record.get("masked_ranges", []):
        mask[start:end] = [False] * (end - start)
    return record["instruction"], mask


def request_task(settings, task):
    return settings["tasks"][task].get("server_task", task)


def response_masks(reply, task_mask, frames, dimensions):
    raw = reply.get("mask", reply.get("action_mask"))
    if raw is None:
        raise RuntimeError("server response does not contain a mask")
    values = np.asarray(raw)
    if values.shape == (dimensions,):
        values = np.broadcast_to(values, (frames, dimensions))
    elif values.ndim == 2 and values.shape[1] == dimensions and values.shape[0] >= frames:
        values = values[:frames]
    else:
        raise RuntimeError("server response mask has an invalid shape")
    if values.dtype != np.bool_ and not np.isin(values, [0, 1]).all():
        raise RuntimeError("server response mask must contain booleans")
    return np.asarray(values, dtype=bool) & np.asarray(task_mask, dtype=bool)


def build_infer_request(settings, task, instruction, task_mask, request_id, session_id,
                        observation, image_sample, kinematics, calibration):
    state = kinematics.state78(observation, calibration)
    mapping = settings["protocol_to_robot"]
    state[27] -= mapping["waist_yaw_offset_rad"]
    state[28] = (state[28] - mapping["waist_pitch_offset_rad"]) / mapping["waist_pitch_scale"]
    state[29] = ((state[29] - mapping["waist_height_offset_m"])
                 / mapping["waist_height_scale"])
    request = build_infer_request_from_state(
        settings, task, instruction, task_mask, request_id, session_id,
        observation["timestamp_ns"], state, image_sample)
    for key in ("source_timestamps_ns", "timestamp_basis"):
        if key in observation:
            request["observation"][key] = observation[key]
    return request


def build_infer_request_from_state(settings, task, instruction, task_mask, request_id,
                                   session_id, timestamp_ns, state, image_sample, state_valid=None):
    if isinstance(image_sample, dict):
        image_fields = dict(image_sample)
        image_fields["images"] = {"head_color": image_fields.pop("head_color")}
    else:
        image_time, image = image_sample
        image_fields = {"images": {"head_color": image}, "image_timestamp_ns": image_time}
    state_mask = np.asarray(task_mask, dtype=bool)
    if state_valid is not None:
        state_mask = state_mask & np.asarray(state_valid, dtype=bool)
    state = np.asarray(state, dtype=np.float32)
    if state.shape != (settings["action_dim"],) or not np.isfinite(state).all():
        raise ValueError("protocol state must be a finite action_dim vector")
    state = state.copy()
    state[~state_mask] = 0
    request = {
        "command": "infer",
        "model": settings.get("model", "default"),
        "task": task,
        "robot_id": settings["robot_id"],
        "mask": task_mask,
        "request_id": request_id,
        "session_id": session_id,
        "observation_timestamp_ns": timestamp_ns,
        "data_contract": settings.get("data_contract"),
        "observation": {
            "state": state.tolist(),
            "state_mask": state_mask.tolist(),
            **image_fields,
        },
    }
    if settings.get("instruction_mode", "request") == "request":
        request["instruction"] = instruction
    return request


def response_rows(reply, request_id, task_mask, steps, settings, observation, kinematics):
    if reply.get("request_id", request_id) != request_id:
        raise RuntimeError("server response request_id mismatch")
    actions = np.asarray(reply.get("actions", reply.get("action_chunk")), dtype=float)
    if actions.ndim != 2 or actions.shape[1] != settings["action_dim"] or len(actions) < steps:
        raise RuntimeError("server returned an invalid action chunk")
    masks = response_masks(reply, task_mask, steps, settings["action_dim"])
    if not np.isfinite(actions[:steps][masks]).all():
        raise RuntimeError("server returned NaN or Inf in an enabled field")
    actions = actions.copy()
    mapping = settings["protocol_to_robot"]
    actions[:steps, 27] += mapping["waist_yaw_offset_rad"]
    actions[:steps, 28] = (actions[:steps, 28] * mapping["waist_pitch_scale"]
                           + mapping["waist_pitch_offset_rad"])
    actions[:steps, 29] = (actions[:steps, 29] * mapping["waist_height_scale"]
                           + mapping["waist_height_offset_m"])
    seed = dict(zip(observation["joint_names"], observation["joint_positions"], strict=True))
    if not all(name in seed for name in JOINTS):
        raise RuntimeError("joint observation is incomplete")
    return kinematics.action_rows(actions[:steps], masks, seed)


def aligned_observation(robot, after_ns=None, stop=None):
    # First establish numeric coverage, then anchor at an actual decoded image.
    deadline = time.monotonic() + 5
    while True:
        if stop is not None:
            check_stop(stop)
        robot.send("observe")
        coverage = robot.wait("observation", timeout=5)
        try:
            t0 = robot.image_anchor(coverage["timestamp_ns"])
            if after_ns is None or t0 > after_ns:
                break
        except RuntimeError:
            if after_ns is None:
                raise
        if time.monotonic() >= deadline:
            raise TimeoutError("no fresh head image after startup arrival")
        time.sleep(.01)
    images = robot.image_history(t0)
    robot.send("observe", timestamp_ns=t0)
    observation = robot.wait("observation", timeout=5)
    return observation, images


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/inference/default.json")
    parser.add_argument("--task", required=True)
    parser.add_argument("--model", required=True, help="deployment name")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_configuration(args.config, args.model)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    settings = config["inference"]
    if args.task not in settings["tasks"]:
        parser.error("unknown task: " + args.task)
    instruction, task_mask = task_spec(settings, args.task)
    steps = settings["execute_steps"] if args.steps is None else args.steps
    if not 1 <= steps <= settings["action_horizon"]:
        parser.error("steps must be within action horizon")
    if "image" not in settings["tasks"][args.task]:
        parser.error("task has no SKYE image crop configured")
    if not args.execute:
        print(f"配置检查完成：task={args.task}, model={args.model}；添加 --execute 后运行。")
        return 0
    models = ROOT / "src/inference/models"
    kinematics = Kinematics(models / "runtime_expanded.urdf", models / "robot_Gento_Sky.xml")
    stop = threading.Event()
    connection = robot = None
    keyboard = threading.Thread(target=q_listener, args=(stop, lambda: robot.send("cancel")
                                if robot is not None and robot.process.poll() is None else None), daemon=True)
    keyboard.start()
    try:
        host, port, connection = server_address(config)
        server = config["server"]
        with PolicyClient(host, port, server["timeout_s"],
                          wire_protocol=server.get("wire_protocol", "json_length_v1")) as policy:
            info = policy.request("info")
            check_info(info, config, args.task, instruction, steps)
            robot = RobotClient(config, task=args.task)
            robot.wait("ready")
            arrival = align_startup(robot, settings["tasks"][args.task], task_mask, settings,
                                    kinematics, config["grippers"], stop, response_rows)
            initial_observation = None
            if arrival is not None:
                robot.reset_image_history(arrival["timestamp_ns"])
                initial_observation = aligned_observation(robot, after_ns=arrival["timestamp_ns"], stop=stop)
            check_stop(stop)
            reset_fields = dict(task=request_task(settings, args.task), model=settings["model"],
                                robot_id=settings["robot_id"], mask=task_mask,
                                data_contract=settings["data_contract"])
            if settings["instruction_mode"] == "request":
                reset_fields["instruction"] = instruction
            reset = policy.request("reset", **reset_fields)
            session_id = reset.get("session_id", uuid.uuid4().hex)
            print("inference 已启动，按 q 结束并依次复位身体、腰部和底盘。", flush=True)
            while not stop.is_set():
                observation, images = initial_observation or aligned_observation(robot)
                initial_observation = None
                check_stop(stop)
                request_id = uuid.uuid4().hex
                request = build_infer_request(
                    settings, request_task(settings, args.task), instruction, task_mask, request_id, session_id,
                    observation, images, kinematics, config["grippers"])
                reply = policy.request_object(request)
                rows = response_rows(reply, request_id, task_mask, steps, settings,
                                     observation, kinematics)
                robot.send("execute", rows=rows, dt=settings["action_dt_s"])
                robot.wait("executed", timeout=steps * settings["action_dt_s"] + 5)
        robot.send("quit")
        robot.wait("finished", timeout=180)
        return 0
    except (KeyboardInterrupt, StartupCancelled):
        stop.set()
        if robot is not None and robot.process.poll() is None:
            robot.send("quit")
            robot.wait("finished", timeout=180)
        return 130
    except Exception:
        if robot is not None and robot.process.poll() is None:
            try:
                robot.send("quit")
                robot.wait("finished", timeout=180)
            except Exception:
                pass
        raise
    finally:
        stop.set()
        keyboard.join(timeout=1)
        if robot is not None:
            robot.close()
        if connection is not None:
            connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
