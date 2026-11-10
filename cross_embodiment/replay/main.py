"""Independent cross-embodiment Gento Skye replay with angle clipping."""
import argparse
import json
from pathlib import Path
import shlex
import signal
import subprocess
import sys

from .dataset import Episode
from .kinematics import GROUPS, JOINTS, build_trajectory, home_seed
from .robot_endpoint import move_duration


PROJECT = Path(__file__).resolve().parents[1]
MODEL = PROJECT / "models/skye"
DEFAULT_CONFIG = PROJECT / "configs/cross_replay.json"


def write_report(path, report):
    if path is None:
        return
    destination = path.resolve()
    allowed = PROJECT.parent / "tests/cross_embodiment"
    if not destination.is_relative_to(allowed):
        raise ValueError("reports must be written under root tests/cross_embodiment")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False))


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--execute", action="store_true", help="send clipped joint trajectory to the real robot")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    episode = Episode(args.episode)
    urdf = MODEL / "reference.urdf"
    mjcf = MODEL / "source.xml"
    seed = home_seed(mjcf, config["waist_height_m"])
    process = None
    try:
        if args.execute:
            endpoint = Path(__file__).with_name("robot_endpoint.py").read_text()
            command = "source /etc/apex/apex_ros_env.sh && exec python3 -B -c " + shlex.quote(endpoint) + " --robot"
            process = subprocess.Popen(
                ["ssh", "-o", "BatchMode=yes", config["robot"], "bash -c " + shlex.quote(command)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, start_new_session=True)
            line = process.stdout.readline()
            if not line:
                raise RuntimeError("robot endpoint did not provide a joint seed")
            live = json.loads(line)
            seed = {name: float(live[name]) for name in JOINTS}
        rows, diagnostics, limits = build_trajectory(
            episode, seed, urdf, mjcf, max_nfev=int(config["max_nfev"]))
        replay_names = tuple(name for _, names, _ in GROUPS for name in names)
        replay_start = tuple(value for group in rows[0][1] for value in group)
        transition_s = move_duration(
            [seed[name] for name in replay_names], replay_start, replay_names, limits,
            float(config["move_to_start_speed_fraction"]),
            float(config["move_to_start_min_duration_s"]))
        report = {
            "episode": str(episode.path),
            "episode_index": episode.episode_index,
            "frames": len(rows),
            "duration_s": rows[-1][0] if rows else 0.0,
            "execute": bool(args.execute),
            "fixed_commands": {"base_vx": 0.0, "base_wz": 0.0, "left_gripper": 0.0, "right_gripper": 0.0},
            "eef_residual_acceptance": "reported only; never rejects after target joint angle/rate clipping",
            "trajectory_start_joint_rad": dict(zip(replay_names, replay_start, strict=True)),
            "start_seed_max_deviation_rad": max(
                abs(value - seed[name]) for name, value in zip(
                    replay_names, replay_start, strict=True)),
            "move_to_start": {
                "enabled_for_execute": True,
                "estimated_duration_s": transition_s,
                "speed_fraction_of_urdf_limit": float(config["move_to_start_speed_fraction"]),
                "control_hz": float(config["move_to_start_control_hz"]),
                "arrival_tolerance_rad": float(config["start_tolerance_rad"]),
            },
            "diagnostics": diagnostics,
        }
        write_report(args.report, report)
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
        if args.execute:
            payload = {
                "rows": rows,
                "limits": {name: limits[name] for name in JOINTS if name in limits},
                "start_tolerance_rad": float(config["start_tolerance_rad"]),
                "move_to_start_speed_fraction": float(config["move_to_start_speed_fraction"]),
                "move_to_start_control_hz": float(config["move_to_start_control_hz"]),
                "move_to_start_min_duration_s": float(config["move_to_start_min_duration_s"]),
                "move_to_start_max_duration_s": float(config["move_to_start_max_duration_s"]),
                "move_to_start_settle_s": float(config["move_to_start_settle_s"]),
            }
            process.stdin.write(json.dumps(payload, allow_nan=False) + "\n")
            process.stdin.flush()
            if process.wait():
                raise RuntimeError("robot-side cross-embodiment replay failed")
    finally:
        if process is not None and process.poll() is None:
            # EOF is the robot endpoint's graceful stop signal. It releases the
            # command/gripper muxes in its finally block before SSH exits.
            try:
                process.stdin.close()
                process.wait(timeout=3)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                process.terminate()
                process.wait()


if __name__ == "__main__":
    try:
        for sig in (signal.SIGHUP, signal.SIGTERM):
            signal.signal(sig, signal.default_int_handler)
        run()
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, RuntimeError, ValueError) as exc:
        sys.exit(str(exc))
