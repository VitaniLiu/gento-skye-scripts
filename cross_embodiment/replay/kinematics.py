"""Independent URDF/MJCF FK and angle-clipped EEF-to-joint retargeting."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


GROUPS = (
    ("flange_L", tuple(f"Joint{i}_L" for i in range(1, 8)), 0),
    ("flange_R", tuple(f"Joint{i}_R" for i in range(1, 8)), 9),
    ("eyes_link", ("Head_J1_Joint", "Head_J2_Joint"), 18),
)
WAIST = tuple(f"waist_joint_{i}" for i in range(1, 4))
JOINTS = GROUPS[0][1] + GROUPS[1][1] + GROUPS[2][1] + WAIST


class Kinematics:
    def __init__(self, urdf, mjcf):
        self.urdf = Path(urdf)
        self.mjcf = Path(mjcf)
        tree = ET.parse(self.urdf).getroot()
        self.joints = []
        self.limits = {}
        for joint in tree.findall("joint"):
            origin = joint.find("origin")
            axis = joint.find("axis")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            transform[:3, 3] = xyz
            self.joints.append((
                joint.get("name"), joint.get("type"), joint.find("parent").get("link"),
                joint.find("child").get("link"), transform,
                np.fromstring(axis.get("xyz") if axis is not None else "1 0 0", sep=" ")))
            limit = joint.find("limit")
            if limit is not None:
                self.limits[joint.get("name")] = tuple(
                    float(limit.get(key)) for key in ("lower", "upper", "velocity"))
        self.required = {joint[0] for joint in self.joints if joint[1] != "fixed"}
        self.extensions = []
        root = ET.parse(self.mjcf).getroot()
        parents = {child: parent for parent in root.iter() for child in parent}
        for name in ("left_tool", "right_tool", "eyes_link"):
            body = next(item for item in root.iter("body") if item.get("name") == name)
            w, x, y, z = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
            transform[:3, 3] = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
            self.extensions.append((parents[body].get("name"), name, transform))

    def forward(self, names, positions, joints=None, extensions=None):
        values = dict(zip(names, positions, strict=True))
        missing = self.required - set(values)
        if missing:
            raise ValueError(f"missing joints: {sorted(missing)}")
        frames = {"base_link": np.eye(4)}
        pending = list(self.joints if joints is None else joints)
        while pending:
            completed = []
            for joint in pending:
                name, kind, parent, child, origin, axis = joint
                if parent not in frames:
                    continue
                motion = np.eye(4)
                if kind in ("revolute", "continuous"):
                    motion[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * values[name]).as_matrix()
                elif kind == "prismatic":
                    motion[:3, 3] = axis * values[name]
                elif kind != "fixed":
                    raise ValueError(f"unsupported joint type: {kind}")
                frames[child] = frames[parent] @ origin @ motion
                completed.append(joint)
            if not completed:
                raise ValueError("disconnected or cyclic kinematic tree")
            pending = [joint for joint in pending if joint not in completed]
        for parent, name, transform in self.extensions if extensions is None else extensions:
            frames[name] = frames[parent] @ transform
        return frames

    def model_fingerprint(self):
        return {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (self.urdf, self.mjcf)
        }


def home_seed(mjcf, waist_height):
    root = ET.parse(mjcf).getroot()
    names = [joint.get("name") for joint in root.find("worldbody").iter("joint") if joint.get("name")]
    key = root.find("./keyframe/key[@name='home']")
    values = dict(zip(names, map(float, key.get("qpos").split()), strict=True))
    values[WAIST[0]] = float(waist_height)
    return {name: values[name] for name in JOINTS}


def _rotation(values):
    x = np.asarray(values[:3], dtype=float)
    y = np.asarray(values[3:6], dtype=float)
    x /= np.linalg.norm(x)
    y -= x * np.dot(x, y)
    y /= np.linalg.norm(y)
    return np.column_stack((x, y, np.cross(x, y)))


def build_trajectory(episode, seed, urdf, mjcf, max_nfev=70):
    kinematics = Kinematics(urdf, mjcf)
    q = np.array([seed[name] for name in JOINTS], dtype=float)
    rows = [[float(t), [], [0.0, 0.0, 0.0, 0.0, None]] for t in episode.relative_times]
    report = {
        "policy": "solve EEF IK, clip joint angle and rate, retain EEF residual without rejection",
        "model_fingerprint": kinematics.model_fingerprint(),
        "arm_eef_frame": "flange (URDF flange_L / flange_R); no tool offset",
        "groups": {},
    }
    all_joints = list(kinematics.joints)
    all_extensions = list(kinematics.extensions)
    offset_by_group = (0, 7, 14)

    for group_index, (body, names, action_start) in enumerate(GROUPS):
        extension = next((item for item in all_extensions if item[1] == body), None)
        extensions = [] if extension is None else [extension]
        parents = {joint[3]: joint for joint in all_joints}
        chain = []
        link = body if extension is None else extension[0]
        while link != "base_link":
            joint = parents[link]
            chain.append(joint)
            link = joint[2]
        chain.reverse()
        slot = slice(offset_by_group[group_index], offset_by_group[group_index] + len(names))
        low, high, speed = np.array([kinematics.limits[name] for name in names]).T
        hard_counts = np.zeros(len(names), dtype=int)
        rate_counts = np.zeros(len(names), dtype=int)
        max_position_error = 0.0
        max_rotation_error = 0.0
        previous = q[slot].copy()

        for index, (row, action) in enumerate(zip(rows, episode.actions, strict=True)):
            values = action[action_start:action_start + 9]
            target_position = values[:3]
            target_rotation = _rotation(values[3:9])
            seed_q = previous.copy()

            def residual(guess):
                q[slot] = guess
                pose = kinematics.forward(JOINTS, q, chain, extensions)[body]
                rotation = Rotation.from_matrix(target_rotation @ pose[:3, :3].T).as_rotvec()
                return np.r_[pose[:3, 3] - target_position, 0.2 * rotation, .001 * (guess - seed_q)]

            # Solve in a deliberately wider mathematical interval, then apply the
            # target robot's own URDF angle limits as the authoritative clamp.
            solver_low = np.minimum(low, -np.pi)
            solver_high = np.maximum(high, np.pi)
            fit = least_squares(
                residual, np.clip(seed_q, solver_low, solver_high),
                bounds=(solver_low, solver_high), max_nfev=max_nfev,
                ftol=1e-6, xtol=1e-6, gtol=1e-7)
            hard_clipped = np.clip(fit.x, low, high)
            hard_counts += ~np.isclose(hard_clipped, fit.x, atol=1e-9)
            if index:
                step = speed * (episode.relative_times[index] - episode.relative_times[index - 1])
                final = np.clip(hard_clipped, previous - step, previous + step)
                rate_counts += ~np.isclose(final, hard_clipped, atol=1e-9)
            else:
                final = hard_clipped
            q[slot] = final
            pose = kinematics.forward(JOINTS, q, chain, extensions)[body]
            position_error = float(np.linalg.norm(pose[:3, 3] - target_position))
            rotation_error = float(Rotation.from_matrix(target_rotation @ pose[:3, :3].T).magnitude())
            max_position_error = max(max_position_error, position_error)
            max_rotation_error = max(max_rotation_error, rotation_error)
            row[1].append(final.tolist())
            previous = final.copy()

        report["groups"][body] = {
            "joint_names": list(names),
            "hard_limit_clipped_frames_per_joint": hard_counts.tolist(),
            "rate_clipped_frames_per_joint": rate_counts.tolist(),
            "max_eef_position_residual_m": max_position_error,
            "max_eef_rotation_residual_rad": max_rotation_error,
        }
    return rows, report, kinematics.limits
