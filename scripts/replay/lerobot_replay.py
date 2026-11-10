#!/usr/bin/env python3
"""Replay LeRobot actions: base_link-relative flange_L/flange_R and eyes_link poses."""

import argparse
import json
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import time


ROBOT = "user@6.6.7.100"
EPISODE = Path("/mnt/apex-data/gento-skye_data/lerobot_clean/gouwu/data/chunk-000/episode_000000.parquet")
JOINT_GROUPS = (
    tuple(f"Joint{i}_L" for i in range(1, 8)),
    tuple(f"Joint{i}_R" for i in range(1, 8)),
    ("Head_J1_Joint", "Head_J2_Joint"),
)
WAIST = tuple(f"waist_joint_{i}" for i in range(1, 4))
JOINTS = sum(JOINT_GROUPS, ()) + WAIST
START_TOLERANCE_RAD = 0.08
DEFAULT_IK_POSITION_TOLERANCE_M = 0.005
MAX_IK_POSITION_TOLERANCE_M = 0.02
DEFAULT_IK_ROTATION_TOLERANCE_RAD = 0.03
MAX_IK_ROTATION_TOLERANCE_RAD = 0.17453292519943295
IK_ROTATION_RESIDUAL_WEIGHT = 0.2
# Maximum first-action IK deviations from nominal Home in the current gouwu
# release, plus 0.02 rad for Home settling, rounded up to 0.005 rad.
GOUWU_START_LIMITS_RAD = dict(zip(sum(JOINT_GROUPS, ()), (
    0.105, 0.080, 0.130, 0.180, 0.175, 0.130, 0.090,
    0.235, 0.105, 0.080, 0.190, 0.100, 0.150, 0.225,
    0.225, 0.305,
), strict=True))


def start_deviations(joints, groups, gouwu=False):
    limits = GOUWU_START_LIMITS_RAD if gouwu else {}
    return [(joint, abs(joints[joint] - target), limits.get(joint, START_TOLERANCE_RAD))
            for joint, target in zip(sum(JOINT_GROUPS, ()), sum(groups, []), strict=True)]


def replay_policy(episode):
    """Use the package schema policy; retain legacy export-only compatibility."""
    meta = episode.parents[2] / "meta"
    schema_path = meta / "schema78.json"
    if schema_path.is_file():
        schema = json.loads(schema_path.read_text())
        if "policy" in schema:
            return schema["policy"]
    return json.loads((meta / "export.json").read_text())["policy"]


def waist_binding(schema):
    bindings = schema.get("source_mapping", {}).get("bindings", [])
    matches = [b for b in bindings if b.get("target") == "action" and b.get("indices") == [29]]
    if len(matches) == 1:
        return matches[0]
    profile_path = Path(__file__).resolve().parents[2] / "configs/replay/native_flange.json"
    profile = json.loads(profile_path.read_text())
    if schema.get("source_mapping", {}).get("schema") != profile["source_schema"]:
        raise ValueError("腰高映射缺失：需要明确的 scale/offset 或受支持的 native profile")
    return profile["waist_binding"]


def replay_ik_tolerances(episode):
    import math

    policy = replay_policy(episode)
    position = float(policy.get("ik_position_tolerance_m", DEFAULT_IK_POSITION_TOLERANCE_M))
    rotation = float(policy.get("ik_rotation_tolerance_rad", DEFAULT_IK_ROTATION_TOLERANCE_RAD))
    if not math.isfinite(position) or not 0 < position <= MAX_IK_POSITION_TOLERANCE_M:
        raise ValueError(
            f"IK 位置容差必须在 (0, {MAX_IK_POSITION_TOLERANCE_M}] m 内：{position}")
    if not math.isfinite(rotation) or not 0 < rotation <= MAX_IK_ROTATION_TOLERANCE_RAD:
        raise ValueError(
            f"IK 旋转容差必须在 (0, {MAX_IK_ROTATION_TOLERANCE_RAD}] rad 内：{rotation}")
    return position, rotation


def read_actions(episode):
    import numpy as np
    import pyarrow.compute as pc
    import pyarrow.parquet as pq

    data = pq.read_table(episode, columns=["action", "timestamp", "validity", "frame_index", "episode_index"])
    quality = pq.read_table(episode.parents[2] / "meta/frame_quality.parquet",
                            columns=["action_valid", "frame_index", "episode_index"])
    quality = quality.filter(pc.equal(quality["episode_index"], data["episode_index"][0]))
    if quality["frame_index"].to_pylist() != data["frame_index"].to_pylist():
        raise ValueError("action 与质量掩码的帧号不一致")
    meta = episode.parents[2] / "meta"
    policy = replay_policy(episode)
    moving_waist = policy["waist_height_enabled"]
    turning = policy.get("base_rotation_policy") == "enabled"
    channels = [*range(27), 30, 54, 77] + ([29] if moving_waist else []) + ([31] if turning else [])
    valid = np.asarray(data["validity"].to_pylist(), bool) & np.asarray(quality["action_valid"].to_pylist(), bool)
    ready = valid[:, channels].all(axis=1)
    first = next((i for i, ok in enumerate(ready) if ok), len(ready))
    if first == len(ready) or not ready[first:].all():
        raise ValueError("所选 episode 缺少连续的完整 action")
    actions = np.asarray(data["action"].to_pylist(), dtype=float)[first:]
    timestamps = np.asarray(data["timestamp"].to_pylist())[first:]
    if not np.isfinite(actions[:, channels]).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError("action 或时间戳无效")

    heights = None
    if moving_waist:
        schema = json.loads((meta / "schema78.json").read_text())
        binding = waist_binding(schema)
        heights = (actions[:, 29] / policy["waist_height_scale"] - binding["offset"]) / binding["scale"]
        if np.any((heights < -1e-7) | (heights > .8600001)):
            raise ValueError("腰高超出模型行程")
        heights = np.clip(heights, 0., .86)
    wz = actions[:, 31] / policy["base_clockwise_scale"] if turning else np.zeros(len(actions))
    return first, actions, timestamps, heights, wz


def trajectory(episode, joints, inputs=None):
    import xml.etree.ElementTree as ET
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    root = Path(__file__).resolve().parents[2]
    from replay_kinematics import Kinematics
    first, actions, timestamps, heights, wz = inputs if inputs is not None else read_actions(episode)
    position_tolerance, rotation_tolerance = replay_ik_tolerances(episode)

    model = root / "models/robot/skye_coordinate_evidence_20261103"
    urdf = model / "runtime_expanded.urdf"
    fk = Kinematics(urdf, model / "robot_Gento_Sky.xml")
    limits = {j.get("name"): j.find("limit").attrib for j in ET.parse(urdf).getroot().findall("joint")
              if j.find("limit") is not None}
    q = np.array([joints[name] for name in JOINTS], dtype=float)
    rows = [[float(t - timestamps[0]), [], [float(a[30]), float(wz[i]), float(a[54]), float(a[77]),
             None if heights is None else float(heights[i])]] for i, (t, a) in enumerate(zip(timestamps, actions))]
    all_joints, all_extensions = fk.joints, fk.extensions
    for k, (body, group) in enumerate(zip(("flange_L", "flange_R", "eyes_link"), JOINT_GROUPS)):
        extension = next((e for e in all_extensions if e[1] == body), None)
        parents = {j[3]: j for j in all_joints}
        chain = []
        link = body if extension is None else extension[0]
        while link != "base_link":
            joint = parents[link]
            chain.append(joint)
            link = joint[2]
        fk.joints = list(reversed(chain))
        fk.extensions = [] if extension is None else [extension]
        offset = (0, 7, 14)[k]
        slot = slice(offset, offset + len(group))
        low, high, speed = np.array([[float(limits[name][f]) for f in ("lower", "upper", "velocity")]
                                     for name in group]).T
        for i, (row, action) in enumerate(zip(rows, actions)):
            if heights is not None:
                q[16] = heights[i]
            v = action[9*k:9*k+9]
            x, y = v[3:6], v[6:9]
            target = np.column_stack((x, y, np.cross(x, y)))
            seed = np.clip(q[slot], low, high)

            def residual(guess):
                q[slot] = guess
                pose = fk.forward(JOINTS, q)[body]
                rotation = Rotation.from_matrix(target @ pose[:3, :3].T).as_rotvec()
                return np.r_[pose[:3, 3] - v[:3],
                             IK_ROTATION_RESIDUAL_WEIGHT * rotation,
                             .001 * (guess-seed)]

            step = speed * (row[0] - rows[i-1][0]) if i else None
            bounds = (np.maximum(low, seed-step), np.minimum(high, seed+step)) if i else (low, high)
            fit = least_squares(residual, seed, bounds=bounds, max_nfev=25,
                                ftol=1e-5, xtol=1e-5, gtol=1e-6)
            if i == 0 and fit.status == 2 and (
                    np.linalg.norm(fit.fun[:3]) > position_tolerance
                    or np.linalg.norm(fit.fun[3:6]) >
                    IK_ROTATION_RESIDUAL_WEIGHT * rotation_tolerance):
                # A near-zero measured seed can trigger ftol while the trust region is still tiny.
                fit = least_squares(residual, seed, bounds=bounds, max_nfev=60,
                                    ftol=None, xtol=1e-5, gtol=1e-6)
            position_error = np.linalg.norm(fit.fun[:3])
            rotation_error = np.linalg.norm(fit.fun[3:6]) / IK_ROTATION_RESIDUAL_WEIGHT
            if (position_error > position_tolerance
                    or rotation_error > rotation_tolerance):
                raise ValueError(
                    f"源帧 {i + first} 的 {body} 目标无法在该帧腰高下复现："
                    f"位置误差 {position_error:.6f}/{position_tolerance:.6f} m，"
                    f"旋转误差 {rotation_error:.6f}/{rotation_tolerance:.6f} rad")
            if i and np.any(np.abs(fit.x - seed) > speed * (row[0] - rows[i-1][0])):
                raise ValueError(f"源帧 {i + first} 的 {body} 关节速度超过模型限制")
            q[slot] = fit.x
            row[1].append(fit.x.tolist())
    return first, rows


def cached_trajectory(episode, joints, inputs):
    import hashlib
    import tempfile

    root = Path(__file__).resolve().parents[2]
    model = root / "models/robot/skye_coordinate_evidence_20261103"
    sources = (Path(__file__), Path(__file__).with_name("replay_kinematics.py"),
               model / "runtime_expanded.urdf", model / "robot_Gento_Sky.xml",
               Path(__file__).resolve().parents[2] / "configs/replay/native_flange.json")
    sources += tuple(p for p in (episode.parents[2] / "meta/schema78.json",
                                episode.parents[2] / "meta/export.json") if p.is_file())
    key = hashlib.sha256(str(inputs[0]).encode())
    for array in inputs[1:]:
        if array is None:
            key.update(b"none")
        else:
            key.update(str((array.shape,array.dtype)).encode())
            key.update(array.tobytes())
    for file in sources:
        with file.open("rb") as stream:
            key.update(hashlib.file_digest(stream, "sha256").digest())
    folder = root / "tests/cache/lerobot_replay"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (key.hexdigest() + ".json")
    geometry = [joints[name] for name in (WAIST if inputs[3] is None else WAIST[1:])]
    try:
        entry = json.loads(path.read_text())
        if max(abs(a-b) for a,b in zip(entry["geometry"],geometry,strict=True)) <= .00025:
            return entry["first"], entry["rows"], True
    except (OSError, ValueError, KeyError, TypeError):
        pass
    first, rows = trajectory(episode, joints, inputs)
    with tempfile.NamedTemporaryFile(mode="w", dir=folder, delete=False) as stream:
        tmp = Path(stream.name)
        json.dump({"geometry":geometry,"first":first,"rows":rows},stream,allow_nan=False)
    tmp.replace(path)
    return first, rows, False


def robot(reset=False, start_height=None, gouwu_start=False):
    import fcntl
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
    from marvin_msgs.srv import Int, JsonCommand
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32, Float64
    from std_srvs.srv import SetBool, Trigger

    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"], domain_id=24,
               signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("lerobot_action_replay")
    joints = {}
    feedback = {}
    joint_time = 0.0

    def receive_joints(msg):
        nonlocal joint_time
        joints.update(zip(msg.name, msg.position))
        joint_time = time.monotonic()

    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(JointState, "/tj/joint_states",
                             receive_joints, sensor_qos)
    node.create_subscription(Jointfeedback, "/tj/info/joint_feedback",
                             lambda m: feedback.update(body=list(m.body_positions), head=m.head_positions[2]),
                             sensor_qos)
    command_mux = node.create_client(Int, "/tj/control/set_input")
    grip_mux = node.create_client(SetBool, "/tj/control/gripper_replay/enable")

    def switch(client, request):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"服务不可用：{client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            raise RuntimeError(f"服务响应超时：{client.srv_name}")
        result = future.result()
        if not result.success:
            raise RuntimeError(f"服务拒绝：{client.srv_name}，{result.message}")
        return result

    def spin_chassis(seconds):
        if select.select([sys.stdin], [], [], 0)[0]:
            raise KeyboardInterrupt
        rclpy.spin_once(node, timeout_sec=seconds)

    def joint_snapshot(require_body=False):
        joints.clear()
        feedback.clear()
        deadline = time.monotonic() + 4
        while not all(name in joints for name in JOINTS) or (require_body and not feedback):
            if time.monotonic() >= deadline:
                raise RuntimeError("未收到完整的机器人关节状态")
            rclpy.spin_once(node, timeout_sec=0.1)
        return joints.copy()

    def reset_robot(episode_height=None):
        config = Path("/home/user/gento-skye_scripts/configs/teleop/waist.json")
        height = float(json.loads(config.read_text())["reset_height_m"]) if episode_height is None else episode_height
        waist_target = (node.create_client(JsonCommand, "/tj/control/waist_reset_to_height")
                        if episode_height is not None else None)
        waist = node.create_client(SetBool, "/tj/control/waist_reset")
        home = node.create_client(Trigger, "/tj/control/go_home")
        body_home = node.create_client(SetBool, "/tj/control/waist_home_passthrough")
        import re
        import xml.etree.ElementTree as ET
        model = ET.parse("/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml")
        names = [j.get("name") for j in model.getroot().find("worldbody").iter("joint") if j.get("name")]
        key = model.find("./keyframe/key[@name='home']")
        target = dict(zip(names, map(float, key.get("qpos").split()), strict=True))
        target = {name: target[name] for name in JOINTS if name != WAIST[0]}

        def wait_for(predicate, seconds, message):
            deadline = time.monotonic() + seconds
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    raise KeyboardInterrupt
                if predicate():
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError(message)
                rclpy.spin_once(node, timeout_sec=0.05)

        current = joint_snapshot()
        height_ready = abs(current[WAIST[0]] - height) <= .002
        home_ready = all(abs(current[j]-v)<=.02 for j,v in target.items())
        if height_ready and home_ready:
            print("腰高和身体已到位，跳过复位", file=sys.stderr, flush=True)
            return
        print(f"复位腰高至 {height:.6f} m", file=sys.stderr, flush=True)
        if waist_target is None:
            switch(waist, SetBool.Request(data=True))
        else:
            switch(waist_target, JsonCommand.Request(command_json=json.dumps({"height_m":height})))
        try:
            joints.pop(WAIST[0], None)
            wait_for(lambda: time.monotonic() - joint_time <= .2
                     and abs(joints.get(WAIST[0], float("inf")) - height) <= .002,
                     46, "腰部未到达遥操设定的复位高度")
            print("腰高已到位", file=sys.stderr, flush=True)
            if all(abs(joints.get(j, float("inf")) - v) <= .02 for j, v in target.items()):
                return
            if not home.wait_for_service(timeout_sec=3):
                raise RuntimeError(f"复位服务不可用：{home.srv_name}")
            print("调用原生身体复位，腰高由腰部控制器保持", file=sys.stderr, flush=True)
            switch(command_mux, Int.Request(data=1))
            switch(body_home, SetBool.Request(data=True))
            try:
                result = switch(home, Trigger.Request())
                match = re.search(r"duration=([0-9.]+)", result.message)
                if not match:
                    raise RuntimeError("原生复位未返回轨迹时长：" + result.message)
                end = time.monotonic() + float(match[1])
                stable = None
                def settled():
                    nonlocal stable
                    now = time.monotonic()
                    within = (now >= end and now - joint_time <= .2 and abs(joints.get(WAIST[0], float("inf")) - height) <= .002
                              and all(abs(joints.get(j, float("inf")) - v) <= .02 for j, v in target.items()))
                    stable = (stable if stable is not None else now) if within else None
                    return stable is not None and now - stable >= .2
                wait_for(settled, float(match[1]) + 8, "原生身体复位后关节反馈未到达 Home")
            finally:
                switch(body_home, SetBool.Request(data=False))
        finally:
            switch(waist, SetBool.Request(data=False))

    armed = grippers = waist_playing = False
    completed = False
    chassis = None
    lock = None
    try:
        if reset:
            lock = open("/home/user/gento-skye_scripts/src/teleop/chassis/.sender.lock", "a")
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("底盘被其他 Teleop 或回放会话占用，请先正常结束该会话") from exc
            reset_robot(start_height)
        seed = joint_snapshot()
        print(json.dumps(seed), flush=True)
        line = sys.stdin.readline()
        if not line:
            return
        rows = json.loads(line)
        current = joint_snapshot(require_body=True)
        if (any(error > limit for _, error, limit in start_deviations(current, rows[0][1], gouwu_start))
                or max(abs(current[j] - seed[j]) for j in WAIST) > 0.002):
            raise RuntimeError("机器人不在回放起始姿态，或准备期间腰部位置发生变化")
        types = (JointcmdArm, JointcmdArm, JointcmdHead, JointcmdBody)
        pubs = [node.create_publisher(typ, "/tj/control/user/joint_cmd_" + suffix, 1)
                for typ, suffix in zip(types, ("A", "B", "head", "body"))]
        # The mux waits for all four groups. Supply a fixed waist pose from feedback.
        body_hold = [current[name] for name in WAIST] + feedback["body"][3:]
        head_hold = feedback["head"]

        def upper(groups):
            stamp = node.get_clock().now().to_msg()
            positions = (groups[0], groups[1], groups[2] + [head_hold], body_hold)
            for pub, typ, angles in zip(pubs, types, positions):
                msg = typ(positions=angles)
                msg.header.stamp = stamp
                pub.publish(msg)

        grip = [node.create_publisher(Float32, "/tj/control/gripper_replay_target" + side, 1) for side in "LR"]
        heartbeat = node.create_publisher(Bool, "/tj/control/gripper_replay/heartbeat", 1)
        switch(command_mux, Int.Request(data=3))
        armed = True
        # Finish the mux's two-second source blend before starting the replay clock.
        for _ in range(44):
            upper(rows[0][1])
            if select.select([sys.stdin], [], [], 0.05)[0]:
                return
        if any(max(abs(row[2][0]), abs(row[2][1])) > 1e-8 for row in rows):
            sys.path.insert(0, "/home/user/gento-skye_scripts/src/replay/raw_data_replay")
            from chassis_replay import Replay
            chassis = Replay(node, None, return_after=True)
            chassis.release_parking(spin_chassis)

        # Acquire the gripper lease after parking release, immediately before streaming.
        if rows[0][2][4] is not None:
            waist_client = node.create_client(SetBool, "/tj/control/waist_playback/enable")
            waist_pub = node.create_publisher(Float64, "/tj/control/waist_playback/height", 1)
            switch(waist_client, SetBool.Request(data=True))
            waist_playing = True
        switch(grip_mux, SetBool.Request(data=True))
        grippers = True
        heartbeat.publish(Bool(data=True))
        if chassis is not None:
            chassis.begin_capture()
        begin = time.monotonic()
        for t, groups, (vx, wz, left, right, height) in rows:
            if select.select([sys.stdin], [], [], max(0, begin + t - time.monotonic()))[0]:
                break
            if time.monotonic() - (begin + t) > 0.1:
                raise RuntimeError("回放延迟超过 100 ms")
            if chassis is not None:
                for _ in range(8):
                    rclpy.spin_once(node, timeout_sec=0)
                if chassis.conflict:
                    raise RuntimeError("底盘有其他指令源，已停止回放")
                parking = chassis.parking.latest
                if max(abs(vx), abs(wz)) > 1e-8 and (
                        parking is None or parking[0] or time.monotonic() - parking[1] >= .2):
                    raise RuntimeError("底盘重新驻车或驻车反馈过期，已停止回放")
            upper(groups)
            heartbeat.publish(Bool(data=True))
            grip[0].publish(Float32(data=left))
            grip[1].publish(Float32(data=right))
            if waist_playing:
                waist_pub.publish(Float64(data=height))
            if chassis is not None:
                command = TwistStamped()
                command.twist.linear.x = vx
                command.twist.angular.z = wz
                chassis.publish(command)
        else:
            last_interval = rows[-1][0] - rows[-2][0] if len(rows) > 1 else 0.05
            completed = not select.select([sys.stdin], [], [], last_interval)[0]
    finally:
        try:
            try:
                if armed:
                    if chassis is not None:
                        chassis.pause()
                    # Apex Home is produced by the native teleop/QP input.
                    # Idle accepts Home requests but does not forward their commands.
                    switch(command_mux, Int.Request(data=1))
            finally:
                try:
                    if grippers:
                        switch(grip_mux, SetBool.Request(data=False))
                finally:
                    if waist_playing:
                        switch(waist_client, SetBool.Request(data=False))
            if reset and completed and not select.select([sys.stdin], [], [], 0)[0]:
                if chassis is not None:
                    deadline = time.monotonic() + 3
                    while True:
                        chassis.pause()
                        sample = chassis.sample()
                        if sample is not None and max(abs(sample.vx), abs(sample.vy)) <= .01 and abs(sample.wz) <= .02:
                            break
                        if time.monotonic() >= deadline:
                            raise RuntimeError("底盘未确认停稳，取消自动复位")
                        spin_chassis(.02)
                print("回放完成，开始复位腰高和身体", file=sys.stderr, flush=True)
                reset_robot()
                if chassis is not None:
                    print("腰高和身体已到位，底盘沿实际轨迹返回起点", file=sys.stderr, flush=True)
                    chassis.return_to_start(spin_chassis)
                    chassis.close()
                    print("底盘已回到回放起点", file=sys.stderr, flush=True)
                print("回放后复位完成", file=sys.stderr, flush=True)
        finally:
            try:
                if chassis is not None:
                    chassis.close()
            finally:
                if lock is not None:
                    lock.close()
                node.destroy_node()
                rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("episode", type=Path, nargs="?", default=EPISODE)
    p.add_argument("--execute", action="store_true", help="发送真实动作。默认只计算关节轨迹")
    args = p.parse_args()
    if not args.episode.is_file():
        p.error(f"找不到 episode：{args.episode}")
    episode = args.episode.resolve()
    gouwu_start = episode.parents[2] == EPISODE.parents[2]
    inputs = read_actions(episode)
    height = float(inputs[3][0]) if inputs[3] is not None else None
    command = "source /etc/apex/apex_ros_env.sh && exec python3 -B -c " + shlex.quote(
        Path(__file__).read_text()) + (" --robot-reset" if args.execute else " --robot")
    if args.execute:
        command += " " + shlex.quote(str(height) if height is not None else "none")
        if gouwu_start:
            command += " gouwu"
    with subprocess.Popen(["ssh", "-o", "BatchMode=yes", ROBOT, "bash -c " + shlex.quote(command)],
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, start_new_session=True) as proc:
        try:
            line = proc.stdout.readline()
            if not line:
                raise RuntimeError("机器人端没有返回关节状态")
            joints = json.loads(line)
            first, rows, hit = cached_trajectory(episode, joints, inputs)
            print("命中关节轨迹缓存" if hit else "已计算并缓存关节轨迹", flush=True)
            deviations = start_deviations(joints, rows[0][1], gouwu_start)
            start_error = max(error for _, error, _ in deviations)
            print(f"源帧 {first} 起共 {len(rows)} 帧，{rows[-1][0]:.2f} 秒；当前最大起始关节误差 {start_error:.3f} rad",
                  flush=True)
            if not args.execute:
                over = [f"{name} {error:.3f}/{limit:.3f} rad"
                        for name, error, limit in deviations if error > limit]
                print("起始关节阈值：" + ("，".join(over) if over else "通过"), flush=True)
            if args.execute:
                proc.stdin.write(json.dumps(rows, allow_nan=False) + "\n")
                proc.stdin.flush()
                if proc.wait():
                    raise RuntimeError("机器人端回放失败")
        finally:
            proc.stdin.close()
            proc.wait()


if __name__ == "__main__":
    try:
        if len(sys.argv)>1 and sys.argv[1] in ("--robot", "--robot-reset"):
            for sig in (signal.SIGHUP, signal.SIGTERM):
                signal.signal(sig, signal.default_int_handler)
            height = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] != "none" else None
            robot(reset=sys.argv[1] == "--robot-reset", start_height=height,
                  gouwu_start=sys.argv[3:] == ["gouwu"])
        else:
            main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, RuntimeError, ValueError) as exc:
        sys.exit(str(exc))
