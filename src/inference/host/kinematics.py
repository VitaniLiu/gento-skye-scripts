"""Inference-local FK/IK: base_link-relative flange poses and eyes_link head pose."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


JOINT_GROUPS = (
    tuple(f"Joint{i}_L" for i in range(1, 8)),
    tuple(f"Joint{i}_R" for i in range(1, 8)),
    ("Head_J1_Joint", "Head_J2_Joint"),
)
WAIST = ("waist_joint_1", "waist_joint_2", "waist_joint_3")
JOINTS = sum(JOINT_GROUPS, ()) + WAIST


class Kinematics:
    def __init__(self, urdf, mjcf):
        tree = ET.parse(urdf).getroot()
        self.joints = []
        self.limits = {}
        for joint in tree.findall("joint"):
            origin, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            matrix[:3, 3] = xyz
            vector = np.fromstring(axis.get("xyz") if axis is not None else "1 0 0", sep=" ")
            self.joints.append((joint.get("name"), joint.get("type"),
                                joint.find("parent").get("link"), joint.find("child").get("link"),
                                matrix, vector))
            if limit is not None:
                self.limits[joint.get("name")] = tuple(float(limit.get(k)) for k in ("lower", "upper", "velocity"))
        root = ET.parse(mjcf).getroot()
        parents = {child: parent for parent in root.iter() for child in parent}
        self.extensions = []
        for name in ("eyes_link",):
            body = next(item for item in root.iter("body") if item.get("name") == name)
            w, x, y, z = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
            matrix[:3, 3] = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
            self.extensions.append((parents[body].get("name"), name, matrix))

    def forward(self, positions):
        values = dict(zip(JOINTS, positions, strict=True))
        frames, pending = {"base_link": np.eye(4)}, self.joints.copy()
        while pending:
            completed = []
            for name, kind, parent, child, origin, axis in pending:
                if parent not in frames:
                    continue
                motion = np.eye(4)
                if kind in ("revolute", "continuous"):
                    motion[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * values[name]).as_matrix()
                elif kind == "prismatic":
                    motion[:3, 3] = axis * values[name]
                frames[child] = frames[parent] @ origin @ motion
                completed.append(name)
            if not completed:
                raise ValueError("kinematic model is disconnected")
            pending = [joint for joint in pending if joint[0] not in completed]
        for parent, name, matrix in self.extensions:
            frames[name] = frames[parent] @ matrix
        return frames

    @staticmethod
    def pose(matrix):
        return np.concatenate((matrix[:3, 3], matrix[:3, :2].T.reshape(6)))

    def state78(self, observation, calibration):
        names = observation["joint_names"]
        positions = observation["joint_positions"]
        by_name = dict(zip(names, positions, strict=True))
        q = np.asarray([by_name[name] for name in JOINTS], dtype=float)
        frames = self.forward(q)
        state = np.zeros(78, dtype=np.float32)
        for start, name in zip((0, 9, 18), ("flange_L", "flange_R", "eyes_link")):
            state[start:start + 9] = self.pose(frames[name])
        state[27:32] = [by_name[WAIST[2]], by_name[WAIST[1]], by_name[WAIST[0]],
                        observation["base_vx"], -observation["base_wz"]]
        for index, side in ((54, "left"), (77, "right")):
            bounds = calibration[side]
            state[index] = np.clip((observation[side + "_gripper"] - bounds["closed"]) /
                                   (bounds["open"] - bounds["closed"]), 0, 1)
        return state

    def action_rows(self, actions, masks, seed):
        actions = np.asarray(actions, dtype=float)
        masks = np.asarray(masks, dtype=bool)
        if actions.shape != masks.shape or actions.ndim != 2 or actions.shape[1] != 78:
            raise ValueError("actions and masks must both have shape [frames, 78]")
        q = np.asarray([seed[name] for name in JOINTS], dtype=float)
        rows = []
        chains = []
        all_joints, all_extensions = self.joints, self.extensions
        for body, group in zip(("flange_L", "flange_R", "eyes_link"), JOINT_GROUPS):
            extension = next((item for item in all_extensions if item[1] == body), None)
            parents = {joint[3]: joint for joint in all_joints}
            chain, link = [], extension[0] if extension is not None else body
            while link != "base_link":
                joint = parents[link]
                chain.append(joint)
                link = joint[2]
            chains.append((body, group, list(reversed(chain)), extension))
        for action, enabled in zip(actions, masks, strict=True):
            groups = []
            if enabled[29]:
                q[16] = action[29]
            if enabled[28]:
                q[17] = action[28]
            if enabled[27]:
                q[18] = action[27]
            for offset, (body, group, chain, extension) in zip((0, 9, 18), chains):
                block = enabled[offset:offset + 9]
                slot = slice((0, 7, 14)[len(groups)], (7, 14, 16)[len(groups)])
                if not block.any():
                    groups.append(q[slot].tolist())
                    continue
                if not block.all():
                    raise ValueError(f"partial pose mask is unsupported for {body}")
                original_joints, original_extensions = self.joints, self.extensions
                self.joints, self.extensions = chain, [extension] if extension is not None else []
                low, high, _speed = np.asarray([self.limits[name] for name in group]).T
                target_columns = action[offset + 3:offset + 9].reshape(2, 3)
                first = target_columns[0]
                n1 = np.linalg.norm(first)
                if n1 < 1e-8:
                    raise ValueError(f"degenerate rotation6d for {body}")
                first = first / n1
                second = target_columns[1] - first * np.dot(first, target_columns[1])
                n2 = np.linalg.norm(second)
                if n2 < 1e-8:
                    raise ValueError(f"degenerate rotation6d for {body}")
                second = second / n2
                target = np.column_stack((first, second, np.cross(first, second)))
                def residual(guess):
                    q[slot] = guess
                    pose = self.forward(q)[body]
                    return np.r_[pose[:3, 3] - action[offset:offset + 3],
                                 .2 * Rotation.from_matrix(target @ pose[:3, :3].T).as_rotvec()]
                result = least_squares(residual, np.clip(q[slot], low, high), bounds=(low, high),
                                       max_nfev=30, ftol=1e-5, xtol=1e-5, gtol=1e-6)
                self.joints, self.extensions = original_joints, original_extensions
                if np.linalg.norm(result.fun[:3]) > .005 or np.linalg.norm(result.fun[3:]) > .006:
                    raise ValueError(f"IK failed for {body}")
                q[slot] = result.x
                groups.append(result.x.tolist())
            rows.append({"groups": groups, "body": [float(q[16]), float(q[17]), float(q[18])],
                         "vx": float(action[30]) if enabled[30] else 0.0,
                         "wz": float(-action[31]) if enabled[31] else 0.0,
                         "left_gripper": float(action[54]) if enabled[54] else None,
                         "right_gripper": float(action[77]) if enabled[77] else None})
        return rows
