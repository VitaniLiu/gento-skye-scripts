#!/usr/bin/env python3
"""Independent robot endpoint for Gento Skye inference."""
import argparse
import base64
import fcntl
import json
import math
from pathlib import Path
import re
import select
import sys
import time

from retrace import RecordedPath, Sample
from alignment import ObservationHistory


def home_duration(message, default=20.0):
    match = re.search(r"duration=([0-9]+(?:\.[0-9]+)?)", str(message))
    return float(match.group(1)) if match else default


def emit(row):
    print(json.dumps(row, allow_nan=False, separators=(",", ":")), flush=True)


class Runtime:
    def __init__(self, config):
        import rclpy
        from foxglove_msgs.msg import CompressedVideo
        from geometry_msgs.msg import TwistStamped
        from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
        from move.msg import State
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, Float32, Float32MultiArray

        self.rclpy = rclpy
        rclpy.init(args=["--ros-args", "--disable-external-lib-logs"],
                   domain_id=config["robot"]["ros_domain_id"])
        self.node = rclpy.create_node("gento_inference")
        self.config = config
        self.observation_history = ObservationHistory()
        self.joints = {}
        self.joint_timestamp_ns = 0
        self.body = None
        self.head_hold = 0.0
        self.grippers = {"left": None, "right": None}
        self.base = None
        self.odom = None
        self.path = RecordedPath()
        self.capture = False
        self.latest_camera_at = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topics = config["topics"]
        self.node.create_subscription(JointState, topics["joints"], self.on_joints, qos)
        self.node.create_subscription(Jointfeedback, topics["body_feedback"], self.on_body, qos)
        self.node.create_subscription(Float32MultiArray, topics["left_gripper_feedback"],
                                      lambda msg: self.on_gripper("left", msg), qos)
        self.node.create_subscription(Float32MultiArray, topics["right_gripper_feedback"],
                                      lambda msg: self.on_gripper("right", msg), qos)
        self.node.create_subscription(CompressedVideo, topics["head_camera"], self.on_camera, qos)
        self.node.create_subscription(State, topics["chassis_state"], self.on_state, qos)
        self.node.create_subscription(Odometry, topics["chassis_odometry"], self.on_odom, qos)
        self.chassis_pub = self.node.create_publisher(TwistStamped, topics["chassis_command"], 1)
        types = (JointcmdArm, JointcmdArm, JointcmdHead, JointcmdBody)
        suffixes = ("A", "B", "head", "body")
        self.upper_pubs = [self.node.create_publisher(kind, "/tj/control/user/joint_cmd_" + suffix, 1)
                           for kind, suffix in zip(types, suffixes)]
        self.upper_types = types
        self.gripper_pubs = {
            "left": self.node.create_publisher(Float32, "/tj/control/gripper_replay_targetL", 1),
            "right": self.node.create_publisher(Float32, "/tj/control/gripper_replay_targetR", 1),
        }
        self.waist_pub = self.node.create_publisher(Float64, "/tj/control/waist_playback/height", 1)
        self.heartbeat_pub = self.node.create_publisher(Bool, "/tj/control/gripper_replay/heartbeat", 1)
        self.Bool, self.Float64 = Bool, Float64
        self.leases = []
        self.node.create_timer(.05, self.heartbeat)
        self.TwistStamped, self.Float32 = TwistStamped, Float32

    def heartbeat(self):
        if "/tj/control/gripper_replay/enable" in self.leases:
            self.heartbeat_pub.publish(self.Bool(data=True))

    def enable_playback(self):
        from std_srvs.srv import SetBool
        for service in ("/tj/control/waist_playback/enable", "/tj/control/gripper_replay/enable"):
            self.call(service, SetBool, SetBool.Request(data=True))
            self.leases.append(service)
        self.heartbeat()

    def disable_playback(self):
        from std_srvs.srv import SetBool
        for service in list(reversed(self.leases)):
            self.call(service, SetBool, SetBool.Request(data=False))
            self.leases.remove(service)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.joint_timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.observation_history.append("joints", self.joint_timestamp_ns, self.joints.copy())

    def on_body(self, msg):
        self.body = list(msg.body_positions)
        self.head_hold = float(msg.head_positions[2])

    def on_gripper(self, side, msg):
        if msg.data:
            self.grippers[side] = float(msg.data[0])
            self.observation_history.append(side + "_gripper", self.node.get_clock().now().nanoseconds, self.grippers[side])

    def on_camera(self, msg):
        now = time.monotonic()
        if now - self.latest_camera_at >= .025:
            self.latest_camera_at = now
            stamp = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
            emit({"type": "camera", "timestamp_ns": stamp,
                  "data": base64.b64encode(bytes(msg.data)).decode("ascii")})

    def on_state(self, msg):
        speed = msg.speed
        self.base = (float(speed.linear.x), float(speed.linear.y), float(speed.angular.z),
                     bool(msg.is_parking_mode), time.monotonic())
        self.observation_history.append("base", self.node.get_clock().now().nanoseconds, (self.base[0], self.base[2]))

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom = Sample(float(pose.position.x), float(pose.position.y),
                           float(pose.orientation.w),
                           *(self.base[:3] if self.base else (0.0, 0.0, 0.0)))
        if self.capture:
            self.path.observe(self.odom)

    def spin(self, seconds=.02):
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def call(self, service, kind, request, timeout=5):
        client = self.node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service unavailable: " + service)
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("service timeout: " + service)
        result = future.result()
        if hasattr(result, "success") and not result.success:
            raise RuntimeError("service rejected: " + service + " " + result.message)
        return result

    def wait(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(message)
            self.spin(.05)

    def home_targets(self):
        import xml.etree.ElementTree as ET
        path = "/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml"
        root = ET.parse(path).getroot()
        names = [j.get("name") for j in root.find("worldbody").iter("joint") if j.get("name")]
        values = map(float, root.find("./keyframe/key[@name='home']").get("qpos").split())
        return dict(zip(names, values))

    def switch_input(self, value):
        from marvin_msgs.srv import Int
        self.call("/tj/control/set_input", Int, Int.Request(data=value))

    def reset_upper(self):
        from std_srvs.srv import Trigger, SetBool
        from marvin_msgs.srv import JsonCommand
        self.disable_playback()
        targets = self.home_targets()
        height = self.config["robot"]["reset_height_m"]
        try:
            self.call("/tj/control/waist_reset_to_height", JsonCommand,
                      JsonCommand.Request(command_json=json.dumps({"height_m": height})))
            self.joints.pop("waist_joint_1", None)
            self.wait(lambda: abs(self.joints.get("waist_joint_1", math.inf) - height) <= .002,
                      46, "waist did not reach inference reset height")
            self.switch_input(1)
            try:
                self.call("/tj/control/waist_home_passthrough", SetBool, SetBool.Request(data=True))
                result = self.call("/tj/control/go_home", Trigger, Trigger.Request(), timeout=5)
                duration = home_duration(result.message)
                end = time.monotonic() + duration
                wanted = {name: value for name, value in targets.items()
                          if name.startswith(("Joint", "Head", "waist_joint_"))
                          and name != "waist_joint_1"}
                self.wait(lambda: time.monotonic() >= end
                          and abs(self.joints.get("waist_joint_1", math.inf) - height) <= .002
                          and all(abs(self.joints.get(name, math.inf) - value) <= .02
                                  for name, value in wanted.items()),
                          duration + 8, "body did not reach Apex Home")
            finally:
                self.call("/tj/control/waist_home_passthrough", SetBool, SetBool.Request(data=False))
        finally:
            self.call("/tj/control/waist_reset", SetBool, SetBool.Request(data=False))
        self.switch_input(3)

    def set_impedance(self):
        from marvin_msgs.srv import Int
        value = self.config["robot"]["impedance_mode"]
        self.call("/tj/control/set_mode", Int, Int.Request(data=value))
        targets = self.home_targets()
        groups = [[targets[name] for name in (f"Joint{i}_L" for i in range(1, 8))],
                  [targets[name] for name in (f"Joint{i}_R" for i in range(1, 8))],
                  [targets["Head_J1_Joint"], targets["Head_J2_Joint"]]]
        body = [self.config["robot"]["reset_height_m"],
                targets["waist_joint_2"], targets["waist_joint_3"]]
        deadline = time.monotonic() + 2.2
        while time.monotonic() < deadline:
            self.publish_upper(groups, body)
            self.spin(.05)

    def release_parking(self):
        from move.srv import SetParkingMode
        request = SetParkingMode.Request(enable=False)
        self.publish_base()
        self.call("/move/SetParkingMode", SetParkingMode, request)
        deadline = time.monotonic() + 3
        while self.base is None or self.base[3] is not False:
            if time.monotonic() >= deadline:
                raise RuntimeError("parking release was not observed")
            self.publish_base()
            self.spin(.02)
        self.capture = True
        self.wait(lambda: self.odom is not None and bool(self.path.points), 2,
                  "chassis origin unavailable")

    def publish_upper(self, groups, body3):
        if self.body is None:
            raise RuntimeError("body feedback unavailable")
        stamp = self.node.get_clock().now().to_msg()
        positions = (groups[0], groups[1], groups[2] + [self.head_hold], body3 + self.body[3:])
        for publisher, kind, values in zip(self.upper_pubs, self.upper_types, positions):
            message = kind(positions=values)
            message.header.stamp = stamp
            publisher.publish(message)

    def publish_base(self, vx=0.0, wz=0.0):
        message = self.TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = vx
        message.twist.angular.z = wz
        self.chassis_pub.publish(message)

    def observation(self, timestamp_ns=None):
        emit(self.observation_history.snapshot(timestamp_ns))

    def execute(self, rows, dt):
        if not rows:
            raise ValueError("execute requires action rows")
        begin = time.monotonic()
        last_stamp = -1.0
        max_late = 0.0
        sent = 0
        for index, row in enumerate(rows):
            stamp = index * float(dt)
            if not math.isfinite(stamp) or stamp < 0 or stamp <= last_stamp:
                raise ValueError("trajectory timestamps must strictly increase")
            last_stamp = stamp
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    if not line or json.loads(line).get("command") == "cancel":
                        self.publish_base()
                        return dict(cancelled=True, sent=sent, max_late_s=max_late)
                    raise RuntimeError("unexpected command during trajectory")
                delay = begin + stamp - time.monotonic()
                if delay <= 0:
                    break
                self.spin(min(.005, delay))
            late = time.monotonic() - begin - stamp
            max_late = max(max_late, late)
            if late > self.config["inference"]["trajectory"]["late_tolerance_s"]:
                self.publish_base()
                raise RuntimeError("trajectory execution late; stale targets were not burst-sent")
            self.publish_upper(row["groups"], row["body"])
            self.waist_pub.publish(self.Float64(data=row["body"][0]))
            self.heartbeat()
            if row["left_gripper"] is not None:
                self.gripper_pubs["left"].publish(self.Float32(data=row["left_gripper"]))
            if row["right_gripper"] is not None:
                self.gripper_pubs["right"].publish(self.Float32(data=row["right_gripper"]))
            self.publish_base(row["vx"], row["wz"])
            sent += 1
        self.publish_base()
        return dict(cancelled=False, sent=sent, max_late_s=max_late)

    def return_base(self):
        self.capture = False
        if self.odom is not None:
            self.path.observe(self.odom)
        controller = self.path.controller()
        deadline = time.monotonic() + 120
        while not controller.finished:
            if time.monotonic() >= deadline or self.odom is None:
                raise RuntimeError("chassis return failed")
            vx, wz = controller.step(self.odom, time.monotonic())
            self.publish_base(vx, wz)
            self.spin(.02)
        for _ in range(10):
            self.publish_base()
            self.spin(.02)

    def close(self):
        try:
            self.publish_base()
            self.disable_playback()
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = json.loads(sys.stdin.readline() if args.config == "-" else Path(args.config).read_text())
    lock_path = "/run/user/1000/gento-skye-inference.lock"
    lock = open(lock_path, "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    runtime = Runtime(config)
    try:
        emit({"type": "stage", "stage": "reset"})
        runtime.reset_upper()
        emit({"type": "stage", "stage": "impedance"})
        runtime.set_impedance()
        emit({"type": "stage", "stage": "parking_release"})
        runtime.release_parking()
        runtime.enable_playback()
        emit({"type": "ready"})
        while True:
            runtime.spin(.01)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            command = json.loads(sys.stdin.readline())
            if command["command"] == "observe":
                runtime.observation(command.get("timestamp_ns"))
            elif command["command"] == "execute":
                result = runtime.execute(command["rows"], command["dt"])
                emit({"type": "executed", **result})
            elif command["command"] == "cancel":
                pass
            elif command["command"] == "quit":
                emit({"type": "stage", "stage": "reset_after_q"})
                runtime.reset_upper()
                emit({"type": "stage", "stage": "return_base"})
                runtime.return_base()
                emit({"type": "finished"})
                break
    except Exception as exc:
        emit({"type": "error", "error": str(exc)})
        return 2
    finally:
        runtime.close()
        lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
