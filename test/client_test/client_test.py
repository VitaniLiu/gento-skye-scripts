#!/usr/bin/env python3
"""Convert a recorded Episode to inference protocol and replay it without a server."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import threading
import uuid

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "src/inference/host"
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(HOST))

from images import ImageHistory
from kinematics import JOINTS, Kinematics  # noqa: E402
from main import (RobotClient, build_infer_request_from_state, jpeg_base64, q_listener,
                  load_configuration, response_rows, task_spec)  # noqa: E402
from processing.conversion import Collector  # noqa: E402
from processing.command_targets import CommandTargets  # noqa: E402
from processing.remote import request as raw_request  # noqa: E402
from processing.schema78 import preflight  # noqa: E402
from processing.task_v21 import policy_for, structural, transform  # noqa: E402


DATA = Path(__file__).resolve().parent / "data"
PROTOCOL_SCHEMA = "gento_skye_inference_protocol_episode_skye_v3"
POSE_CONTRACT = {"reference_frame": "base_link",
                 "eef_frames": {"left": "flange_L", "right": "flange_R", "head": "eyes_link"},
                 "rotation6d": "first_two_columns"}


def dump_line(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":")) + "\n"


def load_config(path, model, task):
    config = load_configuration(path, model)
    settings = config["inference"]
    if task not in settings["tasks"]:
        raise ValueError("unknown task: " + task)
    return config, settings


def make_kinematics():
    models = ROOT / "src/inference/models"
    return Kinematics(models / "runtime_expanded.urdf", models / "robot_Gento_Sky.xml")


class RecordedObservationFK:
    """Recorded feedback FK using production flange definitions, not live data."""
    def __init__(self):
        self.kinematics = make_kinematics()

    def poses78(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("recorded joint names/positions mismatch")
        values = dict(zip(names, positions, strict=True))
        q = np.asarray([values[name] for name in JOINTS], dtype=float)
        if not np.isfinite(q).all():
            raise ValueError("nonfinite recorded joint feedback")
        frames = self.kinematics.forward(q)
        # Collector/task_v21 uses row layout internally, then converts to columns.
        return {name: np.r_[frames[name][:3, 3], frames[name][:2, :3].reshape(6)]
                for name in ("flange_L", "flange_R", "eyes_link")}


class RecordedCommandTargets(CommandTargets):
    def __init__(self, specification, fk):
        super().__init__(specification)
        self.client_fk = fk

    def at(self, t, _processing_fk):
        # Recorded arm/head commands use the same native-wrist FK as the training source.
        return super().at(t, self.client_fk)


class ClientCollector(Collector):
    def __init__(self, schema, scratch):
        super().__init__(schema, scratch)
        self.fk = RecordedObservationFK()
        self.commands = RecordedCommandTargets(schema["command_joint_bundle"], self.fk)


def flange_schema():
    path = ROOT / "configs/processing/action78_posttrain.json"
    schema = json.loads(path.read_text())
    schema["pose_reference"]["eef_definitions"] = dict(POSE_CONTRACT["eef_frames"])
    for binding in schema["bindings"]:
        old = binding.get("eef")
        if old in ("left_tool", "right_tool"):
            binding["eef"] = "flange_L" if old == "left_tool" else "flange_R"
            binding["required_derivation"] = "recorded feedback/command joints -> production native-wrist FK"
    return schema


def protocol_path(task, episode, explicit):
    if explicit is not None:
        return explicit
    return DATA / f"{task}_episode{episode:08d}.skye_v3.protocol.jsonl"


def raw_segments(config, raw_root, task, episode, scratch):
    schema = flange_schema()
    errors = preflight(schema)
    if errors:
        raise RuntimeError("raw Action78 mapping is invalid: " + "; ".join(errors))
    collector = ClientCollector(schema, scratch)
    extraction_config = {
        "robot_ssh": config["robot"]["ssh"],
        "robot_raw_root": str(raw_root),
        "remote_timeout_s": 1800,
        "extract_topics": sorted({binding["topic"] for binding in schema["bindings"]
                                   if binding.get("topic") and binding.get("verified")}),
        "extract_image_topics": sorted(camera["topic"] for camera in schema["cameras"]),
        "extract_bag_subdir": "domain_24",
    }
    groups = schema.get("command_joint_bundle", {}).get("groups", [])
    extraction_config["extract_topics"] = sorted(
        set(extraction_config["extract_topics"]) | {group["topic"] for group in groups})
    source = str(Path(raw_root) / task / f"episode{episode:08d}")
    result = raw_request(extraction_config, "extract", source,
                         progress=lambda message: print("raw:", message, flush=True),
                         record_handler=collector)
    if result["errors"]:
        raise RuntimeError("raw Episode extraction was incomplete: " + repr(result["errors"][:5]))
    segments, alignment = collector.finish(result["timeline"]["origin_ns"])
    rows = [row for segment in segments for row in segment]
    if not rows:
        raise RuntimeError("raw Episode produced no protocol frames")
    return schema, rows, alignment, source


def image_sample(row):
    value = row["observation.images.head"]
    path = Path(value["path"] if isinstance(value, dict) else value)
    with Image.open(path) as image:
        array = np.asarray(image.convert("RGB"))
    timestamp = int(row["source.camera_head_time_ns"][0])
    return timestamp, jpeg_base64(array)


def converted_vector(row, key, side, policy, policy_mask, task_mask=None):
    valid = np.asarray(row[f"quality.{side}_valid"], dtype=bool).copy() & policy_mask
    if task_mask is not None:
        valid &= np.asarray(task_mask, dtype=bool)
    values = transform(row[key], policy_mask, policy, valid)
    return values, valid


def convert_episode(args, config, settings):
    output = protocol_path(args.task, args.episode, args.protocol_file)
    output.parent.mkdir(parents=True, exist_ok=True)
    scratch = DATA / "cache" / (args.task + f"_{args.episode:08d}_" + uuid.uuid4().hex)
    scratch.mkdir(parents=True)
    partial = output.with_name(output.name + ".partial")
    instruction, task_mask = task_spec(settings, args.task)
    try:
        schema, rows, alignment, source = raw_segments(
            config, args.raw_root, args.task, args.episode, scratch)
        policy = policy_for(args.task)
        policy_mask = structural(policy).astype(bool)
        horizon = settings["action_horizon"]
        history = ImageHistory(settings["tasks"][args.task]["image"], settings["image_history_offsets_s"])
        image_cursor = 0
        last_image_time = -1
        header = {
            "kind": "header",
            "schema": PROTOCOL_SCHEMA,
            "pose_contract": POSE_CONTRACT,
            "data_contract": settings["data_contract"],
            "image_spec": settings["tasks"][args.task]["image"],
            "pose_sources": {"state": "recorded joint feedback -> client flange FK",
                             "arm_action": "recorded controller joint targets -> native-wrist FK",
                             "head_action": "recorded controller joint targets -> eyes_link FK"},
            "task": args.task,
            "source_episode": source,
            "action_dim": settings["action_dim"],
            "action_dt_s": settings["action_dt_s"],
            "action_horizon": horizon,
            "frame_count": len(rows),
            "chunk_count": (len(rows) + horizon - 1) // horizon,
            "task_mask": task_mask,
            "raw_mapping_schema": schema["schema"],
            "alignment": alignment,
        }
        session_id = "recorded-episode-" + uuid.uuid4().hex
        with partial.open("w", encoding="utf-8") as stream:
            stream.write(dump_line(header))
            for start in range(0, len(rows), horizon):
                chunk = rows[start:start + horizon]
                state, _state_mask = converted_vector(
                    chunk[0], "observation.state", "state", policy, policy_mask)
                request_id = uuid.uuid4().hex
                t0 = int(chunk[0]["source.grid_time_ns"][0])
                while image_cursor <= start:
                    item = rows[image_cursor]
                    stamp = int(item["source.camera_head_time_ns"][0])
                    if last_image_time < stamp <= t0:
                        value = item["observation.images.head"]
                        path = Path(value["path"] if isinstance(value, dict) else value)
                        with Image.open(path) as image:
                            history.append(stamp, np.asarray(image.convert("RGB")))
                        last_image_time = stamp
                    image_cursor += 1
                request = build_infer_request_from_state(
                    settings, args.task, instruction, task_mask, request_id, session_id,
                    t0, state, history.sample(t0), state_valid=_state_mask)
                actions, masks = [], []
                for row in chunk:
                    action, mask = converted_vector(
                        row, "action", "action", policy, policy_mask, task_mask)
                    actions.append(action.tolist())
                    masks.append(mask.tolist())
                response = {
                    "ok": True,
                    "request_id": request_id,
                    "actions": actions,
                    "mask": masks,
                }
                stream.write(dump_line({
                    "kind": "exchange",
                    "start_frame": start,
                    "frame_count": len(chunk),
                    "request": request,
                    "response": response,
                }))
        partial.replace(output)
        print("saved:", output)
        return output
    finally:
        if partial.exists():
            partial.unlink()
        shutil.rmtree(scratch, ignore_errors=True)


def read_protocol(path):
    stream = path.open(encoding="utf-8")
    try:
        first = stream.readline()
        if not first:
            raise ValueError("protocol file is empty")
        header = json.loads(first)
        if header.get("kind") != "header" or header.get("schema") != PROTOCOL_SCHEMA:
            raise ValueError("unsupported protocol file")
        if header.get("pose_contract") != POSE_CONTRACT:
            raise ValueError("protocol must contain base_link-relative flange poses")
        yield header
        for line_number, line in enumerate(stream, 2):
            record = json.loads(line)
            if record.get("kind") != "exchange":
                raise ValueError(f"invalid protocol record at line {line_number}")
            yield record
    finally:
        stream.close()


def observe(robot):
    robot.send("observe")
    return robot.wait("observation", timeout=5)


def finish_robot(robot, reset=True):
    if robot is None:
        return
    original_error = sys.exc_info()[1]
    try:
        if reset and robot.process.poll() is None:
            robot.send("quit")
            robot.wait("finished", timeout=180)
    except Exception as exc:
        if original_error is None:
            raise
        print("cleanup failed: " + str(exc), file=sys.stderr)
    finally:
        robot.close()


def replay_file(args, config, settings, path):
    records = read_protocol(path)
    header = next(records)
    instruction, task_mask = task_spec(settings, args.task)
    del instruction
    if header.get("data_contract") != settings["data_contract"]:
        raise ValueError("protocol data_contract differs from client")
    if header.get("image_spec") != settings["tasks"][args.task].get("image"):
        raise ValueError("protocol image preprocessing differs from client")
    if header["task"] != args.task:
        raise ValueError("protocol task differs from --task")
    if header["action_dim"] != settings["action_dim"]:
        raise ValueError("protocol action dimension differs from client")
    if header["task_mask"] != task_mask:
        raise ValueError("protocol task mask differs from client config")
    if header["action_dt_s"] != settings["action_dt_s"]:
        raise ValueError("protocol action period differs from client config")
    kinematics = make_kinematics()
    stop = threading.Event()
    keyboard = threading.Thread(target=q_listener, args=(stop,), daemon=True)
    keyboard.start()
    robot = None
    robot_ready = False
    executed = 0
    try:
        robot = RobotClient(config, decode_camera=False)
        robot.wait("ready")
        robot_ready = True
        print("protocol replay started; press q to stop and reset", flush=True)
        for record in records:
            if stop.is_set():
                break
            request = record["request"]
            response = record["response"]
            count = record["frame_count"]
            if count != len(response.get("actions", [])) or count != len(response.get("mask", [])):
                raise ValueError("protocol action/mask count mismatch")
            rows = response_rows(response, request["request_id"], task_mask, count,
                                 settings, observe(robot), kinematics)
            robot.send("execute", rows=rows, dt=header["action_dt_s"])
            robot.wait("executed", timeout=count * header["action_dt_s"] + 5)
            executed += count
        print(f"protocol replay finished: {executed}/{header['frame_count']} frames", flush=True)
    finally:
        stop.set()
        finish_robot(robot, reset=robot_ready)
        keyboard.join(timeout=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "configs/inference/default.json")
    parser.add_argument("--task", required=True)
    parser.add_argument("--model", default="default")
    parser.add_argument("--raw-root", type=Path,
                        default=Path("/home/user/gento-skye_raw_data"))
    parser.add_argument("--execute", action="store_true",
                        help="allow replay/run to reset and move the robot")
    subparsers = parser.add_subparsers(dest="command", required=True)
    convert_parser = subparsers.add_parser(
        "convert", help="convert one complete raw Episode to a protocol JSONL file")
    convert_parser.add_argument("--episode", type=int, required=True)
    convert_parser.add_argument("--protocol-file", type=Path)
    replay_parser = subparsers.add_parser(
        "replay", help="execute a previously converted protocol JSONL file")
    replay_parser.add_argument("--protocol-file", type=Path, required=True)
    run_parser = subparsers.add_parser(
        "run", help="convert one raw Episode and immediately replay the protocol file")
    run_parser.add_argument("--episode", type=int, required=True)
    run_parser.add_argument("--protocol-file", type=Path)
    args = parser.parse_args(argv)
    config, settings = load_config(args.config, args.model, args.task)
    if args.command in ("replay", "run") and not args.execute:
        parser.error("replay and run require --execute")
    if args.command == "convert":
        convert_episode(args, config, settings)
    elif args.command == "replay":
        replay_file(args, config, settings, args.protocol_file)
    else:
        path = convert_episode(args, config, settings)
        replay_file(args, config, settings, path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
