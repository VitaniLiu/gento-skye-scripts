"""Causal numeric observation snapshots in the robot ROS clock domain."""
from collections import deque


class ObservationHistory:
    def __init__(self, capacity=1024):
        self.streams = {key: deque(maxlen=capacity) for key in
                        ("joints", "left_gripper", "right_gripper", "base")}

    def append(self, key, timestamp_ns, value):
        rows = self.streams[key]
        if rows and timestamp_ns <= rows[-1][0]:
            return
        rows.append((int(timestamp_ns), value))

    def snapshot(self, timestamp_ns=None):
        if any(not rows for rows in self.streams.values()):
            raise RuntimeError("robot observation is incomplete")
        t0 = min(rows[-1][0] for rows in self.streams.values()) if timestamp_ns is None else int(timestamp_ns)
        chosen = {}
        for key, rows in self.streams.items():
            item = next((row for row in reversed(rows) if row[0] <= t0), None)
            if item is None:
                raise RuntimeError("observation history does not cover image time: " + key)
            chosen[key] = item
        joints = chosen["joints"][1]
        base = chosen["base"][1]
        return {"type": "observation", "timestamp_ns": t0,
                "joint_names": list(joints), "joint_positions": list(joints.values()),
                "left_gripper": chosen["left_gripper"][1],
                "right_gripper": chosen["right_gripper"][1],
                "base_vx": base[0], "base_wz": base[1],
                "source_timestamps_ns": {key: value[0] for key, value in chosen.items()},
                "timestamp_basis": {"joints": "message_header_ros_time",
                                    "left_gripper": "robot_receive_ros_time",
                                    "right_gripper": "robot_receive_ros_time",
                                    "base": "robot_receive_ros_time"}}
