"""Versioned Action78 mapping primitives; no hardware or raw-data writes."""
from pathlib import Path
import numpy as np

VERSION = "action78_pre_aligned_body5_20261102_v1"

def fail(function, reason):
    raise ValueError(f"{Path(__file__).resolve()}:{function}: {reason}")

def rigid(matrix):
    m = np.asarray(matrix, dtype=float)
    if (m.shape != (4, 4) or not np.isfinite(m).all()
        or not np.allclose(m[3], [0, 0, 0, 1])
        or not np.allclose(m[:3,:3].T @ m[:3,:3], np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(m[:3,:3]), 1)):
        fail("rigid", "expected finite proper rigid 4x4 transform")
    return m

def window_pose(world_from_eef, world_from_camera_at_start):
    """Inputs must be evidenced, time-matched transforms; C0 stays fixed."""
    we, wc = rigid(world_from_eef), rigid(world_from_camera_at_start)
    rotation = wc[:3,:3].T @ we[:3,:3]
    position = wc[:3,:3].T @ (we[:3,3] - wc[:3,3])
    return np.concatenate((position, rotation[:2].reshape(6))).astype(np.float32)

def inverse_window_pose(pose9, world_from_camera_at_start):
    v = np.asarray(pose9, dtype=float)
    if v.shape != (9,) or not np.isfinite(v).all():
        fail("inverse_window_pose", "expected finite pose[9]")
    a, b = v[3:6], v[6:9]
    if not np.isclose(a@a, 1, atol=1e-6) or not np.isclose(b@b, 1, atol=1e-6) or not np.isclose(a@b, 0, atol=1e-6):
        fail("inverse_window_pose", "rotation rows must be orthonormal")
    ce = np.eye(4)
    ce[:3,:3] = np.stack((a, b, np.cross(a,b)))
    ce[:3,3] = v[:3]
    return rigid(world_from_camera_at_start) @ ce

def gripper01(value, calibration):
    """Linear mapping is available only with explicit calibrated linear evidence."""
    if not calibration.get("verified") or not calibration.get("evidence") or calibration.get("method") != "linear_endpoints":
        fail("gripper01", "native-to-[0,1] linear calibration unverified")
    try:
        v, closed, opened = map(float, (value, calibration["closed"], calibration["open"]))
    except (KeyError, TypeError, ValueError):
        fail("gripper01", "finite native value and closed/open endpoints required")
    if not np.isfinite([v, closed, opened]).all() or opened == closed:
        fail("gripper01", "invalid native value or identical/nonfinite endpoints")
    result = (v-closed)/(opened-closed)
    if calibration.get("out_of_range")=="saturate_with_report":
        result=float(np.clip(result,0.,1.))
    elif not 0 <= result <= 1:
        fail("gripper01", f"outside calibrated range: value={v}, closed={closed}, open={opened}")
    return np.float32(result)

def masked_values(values, static_mask, valid_mask):
    """Caller retains per-element invalid reasons; missing never becomes valid."""
    a = np.asarray(values, dtype=np.float32)
    applicable = np.asarray(static_mask)
    valid = np.asarray(valid_mask)
    if a.shape[-1:] != (78,) or applicable.shape != (78,) or valid.shape != a.shape:
        fail("masked_values", "expected values[...,78], bool static[78], bool valid matching values")
    if applicable.dtype != np.bool_ or valid.dtype != np.bool_:
        fail("masked_values", "masks must be bool, not numeric validity flags")
    effective = valid & applicable
    if not np.isfinite(a[effective]).all():
        fail("masked_values", "NaN/Inf marked valid; record missing reason and clear mask first")
    return np.where(effective, a, np.float32(0)), effective

def export_blockers(schema):
    prefix = f"{Path(__file__).resolve()}:export_blockers: "
    problems = []
    if schema.get("schema") != VERSION or schema.get("fps") != 20:
        problems.append("schema/version or requested 20Hz differs")
    if schema.get("rotation6d",{}).get("order") != ["R00","R01","R02","R10","R11","R12"]:
        problems.append("rotation6D must use first two rows")
    if schema.get("pose_reference",{}).get("verified") is not True:
        problems.append("C0 RGB optical identity, EEF definitions and timestamped transform chain unverified")
    for binding in schema.get("bindings",[]):
        if not binding.get("verified") or not binding.get("evidence"):
            problems.append(f"{binding['target']}{binding['indices']}: source/calibration unverified")
        if not binding.get("clock",{}).get("verified"):
            problems.append(f"{binding['target']}{binding['indices']}: clock mapping unverified")
    problems.append("window exporter not implemented: T=32 anchor/stride/label timing, bool per-frame masks, mask-aware statistics and self-contained inverse transforms required; legacy base_link exporter prohibited")
    return [prefix + p for p in problems]
