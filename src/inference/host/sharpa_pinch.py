"""Sharpa Wave fingertip FK and binary Gento gripper mapping."""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


HAND_NAMES = (
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
)


class SharpaHand:
    def __init__(self, urdf, side):
        if side not in ("left", "right"):
            raise ValueError("Sharpa hand side must be left or right")
        self.side = side
        root = ET.parse(urdf).getroot()
        self.root_link = side + "_hand_C_MC"
        self.thumb_tip = side + "_thumb_fingertip"
        self.index_tip = side + "_index_fingertip"
        links = {link.get("name") for link in root.findall("link")}
        if not {self.root_link, self.thumb_tip, self.index_tip}.issubset(links):
            raise ValueError("Sharpa URDF is missing required palm or fingertip links")
        self.joints = []
        names = set()
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            axis_node = joint.find("axis")
            axis = np.fromstring(axis_node.get("xyz", "1 0 0"), sep=" ") \
                if axis_node is not None else np.array([1.0, 0.0, 0.0])
            name = joint.get("name")
            names.add(name)
            self.joints.append((name, joint.get("type"),
                                joint.find("parent").get("link"),
                                joint.find("child").get("link"), transform, axis))
        expected = {side + "_" + name for name in HAND_NAMES}
        if not expected.issubset(names):
            raise ValueError("Sharpa URDF joint names do not match the 78D hand layout")

    def fingertip_distance(self, values):
        values = np.asarray(values, dtype=float)
        if values.shape != (22,) or not np.isfinite(values).all():
            raise ValueError("Sharpa hand values must be a finite 22D vector")
        positions = {self.side + "_" + name: value
                     for name, value in zip(HAND_NAMES, values, strict=True)}
        frames = {self.root_link: np.eye(4)}
        pending = self.joints.copy()
        while pending:
            remaining = []
            for name, kind, parent, child, origin, axis in pending:
                if parent not in frames:
                    remaining.append((name, kind, parent, child, origin, axis))
                    continue
                motion = np.eye(4)
                if kind in ("revolute", "continuous"):
                    norm = np.linalg.norm(axis)
                    if norm < 1e-12:
                        raise ValueError("Sharpa URDF contains a zero joint axis")
                    motion[:3, :3] = Rotation.from_rotvec(
                        axis / norm * positions[name]).as_matrix()
                elif kind == "prismatic":
                    motion[:3, 3] = axis * positions[name]
                frames[child] = frames[parent] @ origin @ motion
            if len(remaining) == len(pending):
                raise ValueError("Sharpa URDF is disconnected from its palm link")
            pending = remaining
        return float(np.linalg.norm(frames[self.thumb_tip][:3, 3]
                                    - frames[self.index_tip][:3, 3]))


class SharpaPinchMapper:
    def __init__(self, left_urdf, right_urdf):
        self.hands = {
            "left": SharpaHand(left_urdf, "left"),
            "right": SharpaHand(right_urdf, "right"),
        }

    def map_actions(self, actions, config):
        values = np.asarray(actions, dtype=float).copy()
        if values.ndim != 2 or values.shape[1] != 78:
            raise ValueError("Sharpa action rows must have shape [frames, 78]")
        threshold = float(config["threshold_m"])
        closed = float(config.get("closed_value", 0.0))
        opened = float(config.get("open_value", 1.0))
        if not np.isfinite(threshold) or threshold <= 0:
            raise ValueError("Sharpa pinch threshold must be positive and finite")
        if not (0.0 <= closed <= 1.0 and 0.0 <= opened <= 1.0):
            raise ValueError("Gento gripper values must be within [0, 1]")
        diagnostics = {"threshold_m": threshold, "closed_value": closed,
                       "open_value": opened, "sides": {}}
        for side, start, destination in (("left", 32, 54), ("right", 55, 77)):
            distances = np.asarray([
                self.hands[side].fingertip_distance(row[start:start + 22])
                for row in values
            ])
            commands = np.where(distances < threshold, closed, opened)
            values[:, destination] = commands
            diagnostics["sides"][side] = {
                "distance_m": distances.tolist(),
                "command": commands.tolist(),
            }
        return values, diagnostics
