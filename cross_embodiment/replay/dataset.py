"""Standalone decoder for cross-embodiment 78D replay episodes."""
from pathlib import Path
import json

import numpy as np
import pyarrow.compute as pc
import pyarrow.parquet as pq


EEF = tuple(range(27))
FIXED_ZERO = (30, 31, 54, 77)
HANDS = tuple(range(32, 54)) + tuple(range(55, 77))
REQUIRED = EEF + FIXED_ZERO


class Episode:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.root = self.path.parents[2]
        meta = self.root / "meta"
        table = pq.read_table(
            self.path,
            columns=["action", "timestamp", "validity", "frame_index", "episode_index"])
        quality = pq.read_table(
            meta / "frame_quality.parquet",
            columns=["action_valid", "frame_index", "episode_index"])
        quality = quality.filter(pc.equal(quality["episode_index"], table["episode_index"][0]))
        frames = np.asarray(table["frame_index"].to_pylist(), dtype=int)
        quality_frames = np.asarray(quality["frame_index"].to_pylist(), dtype=int)
        if not np.array_equal(frames, quality_frames):
            raise ValueError("action and frame_quality frame indexes differ")

        self.actions = np.asarray(table["action"].to_pylist(), dtype=float)
        self.validity = np.asarray(table["validity"].to_pylist(), dtype=bool)
        self.quality = np.asarray(quality["action_valid"].to_pylist(), dtype=bool)
        self.timestamps = np.asarray(table["timestamp"].to_pylist(), dtype=float)
        self.frames = frames
        self.episode_index = int(table["episode_index"][0].as_py())
        if self.actions.shape != (len(frames), 78):
            raise ValueError("expected N x 78 actions")
        if self.validity.shape != self.actions.shape or self.quality.shape != self.actions.shape:
            raise ValueError("mask shapes do not match actions")
        if not np.isfinite(self.actions[:, REQUIRED]).all():
            raise ValueError("required actions contain nonfinite values")
        if len(self.timestamps) > 1 and np.any(np.diff(self.timestamps) <= 0):
            raise ValueError("timestamps are not strictly increasing")
        if not (self.validity[:, REQUIRED] & self.quality[:, REQUIRED]).all():
            raise ValueError("required replay dimensions are not continuously valid")
        if self.validity[:, HANDS].any():
            raise ValueError("dexterous-hand dimensions must remain masked")
        if np.any(np.abs(self.actions[:, FIXED_ZERO]) > 1e-8):
            raise ValueError("base and gripper commands must be fixed at zero")

        self.policy = json.loads((meta / "export.json").read_text())["policy"]
        if self.policy.get("waist_height_enabled"):
            raise ValueError("independent cross-embodiment replay currently requires fixed waist")
        if self.policy.get("base_rotation_policy") != "locked_zero":
            raise ValueError("base rotation must be locked zero")
        position = float(self.policy.get("ik_position_tolerance_m", 0.02))
        rotation = float(self.policy.get("ik_rotation_tolerance_rad", np.deg2rad(10)))
        if not 0 < position <= 0.02 or not 0 < rotation <= np.deg2rad(10) + 1e-12:
            raise ValueError("dataset tolerance policy exceeds 20 mm / 10 degrees")

    @property
    def relative_times(self):
        return self.timestamps - self.timestamps[0]
