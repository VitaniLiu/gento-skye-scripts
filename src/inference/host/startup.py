"""Once-per-launch task DATA-state alignment, using existing inference IK/planner.

No endpoint construction or policy protocol changes. Base velocities are never
startup targets. Arrival checks use measured native base-link poses, enabled
waist coordinates and calibrated gripper fractions; finger fields are ignored.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from trajectory import plan_rows
from kinematics import JOINTS


class StartupCancelled(Exception):
    """Startup was stopped; policy reset/inference must not run."""


def check_stop(stop):
    if stop.is_set():
        raise StartupCancelled("startup cancelled")


def startup_target(spec, task_mask):
    target = np.asarray(spec["state"], dtype=float).copy()
    mask = np.asarray(task_mask, dtype=bool).copy()
    if target.shape != (78,) or mask.shape != (78,):
        raise ValueError("startup state/mask must contain 78 fields")
    # These are instantaneous velocities, never a chassis position to restore.
    target[30:32] = 0
    mask[30:] = False
    mask[54] = bool(task_mask[54])
    mask[77] = bool(task_mask[77])
    if not np.isfinite(target[mask]).all():
        raise ValueError("startup state contains nonfinite enabled fields")
    for index in (54, 77):
        if mask[index] and not 0 <= target[index] <= 1:
            raise ValueError("startup gripper target must be a fraction in [0,1]")
    return target, mask


def rotation_from_pose(pose):
    columns = np.asarray(pose[3:9]).reshape(2, 3)
    first = columns[0]
    if np.linalg.norm(first) < 1e-8:
        raise ValueError("degenerate startup rotation6d")
    first = first / np.linalg.norm(first)
    second = columns[1] - first * np.dot(first, columns[1])
    if np.linalg.norm(second) < 1e-8:
        raise ValueError("degenerate startup rotation6d")
    second = second / np.linalg.norm(second)
    return np.column_stack((first, second, np.cross(first, second)))


def arrival_errors(observation, target, mask, settings, kinematics, calibration, spec):
    measured = kinematics.state78(observation, calibration).astype(float)
    mapping = settings["protocol_to_robot"]
    native = target.copy()
    native[27] += mapping["waist_yaw_offset_rad"]
    native[28] = native[28] * mapping["waist_pitch_scale"] + mapping["waist_pitch_offset_rad"]
    native[29] = native[29] * mapping["waist_height_scale"] + mapping["waist_height_offset_m"]
    tolerances = spec["arrival_tolerances"]
    errors = {}
    for offset, body in ((0, "left"), (9, "right"), (18, "head")):
        block = mask[offset:offset+9]
        if not block.any():
            continue
        if not block.all():
            raise ValueError("partial startup pose mask is unsupported")
        actual, expected = measured[offset:offset+9], native[offset:offset+9]
        position = float(np.linalg.norm(actual[:3] - expected[:3]))
        rotation = float(Rotation.from_matrix(rotation_from_pose(expected) @ rotation_from_pose(actual).T).magnitude())
        position_limit = tolerances["head_position_m" if body == "head" else "arm_position_m"]
        if not np.isfinite(position) or position > position_limit:
            errors[body + "_position_m"] = position
        if not np.isfinite(rotation) or rotation > tolerances["rotation_rad"]:
            errors[body + "_rotation_rad"] = rotation
    for index, name, tolerance in ((27, "waist_yaw_rad", "waist_angle_rad"),
                                  (28, "waist_pitch_rad", "waist_angle_rad"),
                                  (29, "waist_height_m", "waist_height_m"),
                                  (54, "left_gripper", "gripper_fraction"),
                                  (77, "right_gripper", "gripper_fraction")):
        if mask[index]:
            error = float(abs(measured[index] - native[index]))
            if not np.isfinite(error) or error > tolerances[tolerance]:
                errors[name] = error
    return errors


def align_startup(robot, record, task_mask, settings, kinematics, calibration, stop, convert_rows):
    spec = record.get("startup")
    if spec is None:
        return None
    check_stop(stop)
    target, mask = startup_target(spec, task_mask)
    robot.send("observe")
    observation = robot.wait("observation", timeout=5)
    check_stop(stop)
    if not arrival_errors(observation, target, mask, settings, kinematics, calibration, spec):
        print("startup: already at task DATA first-frame state", flush=True)
        return observation
    print("startup: aligning to task DATA first-frame state", flush=True)
    rows = convert_rows(dict(actions=[target.tolist()], mask=mask.tolist()), "startup", mask.tolist(),
                        1, settings, observation, kinematics)
    # Reject an unreachable target before commanding any transition.
    solved = dict(observation, joint_names=list(JOINTS),
                  joint_positions=sum(rows[0]["groups"], []) + rows[0]["body"])
    for side in ("left", "right"):
        value = rows[0][side + "_gripper"]
        if value is not None:
            bounds = calibration[side]
            solved[side + "_gripper"] = bounds["closed"] + value * (bounds["open"] - bounds["closed"])
    errors = arrival_errors(solved, target, mask, settings, kinematics, calibration, spec)
    if errors:
        raise RuntimeError("startup solved target fails arrival tolerances: " + str(errors))
    plan = plan_rows(rows, observation, kinematics, settings, calibration, startup_transition=True)
    dt = float(settings["trajectory"]["servo_dt_s"])
    if abs(dt - .01) > 1e-12:
        raise ValueError("startup alignment requires the existing 100 Hz planner")
    check_stop(stop)
    # Runtime indexes rows at uniform dt; its final partial planner interval is
    # conservatively stretched to dt, preserving the planner rate bounds.
    robot.send("execute", rows=plan.rows, dt=dt)
    executed = robot.wait("executed", timeout=len(plan.rows) * dt + 10)
    if executed.get("cancelled"):
        raise StartupCancelled("startup trajectory cancelled")
    check_stop(stop)
    if executed.get("sent") != len(plan.rows):
        raise RuntimeError("startup trajectory was not completely executed")
    deadline = time.monotonic() + float(spec["arrival_timeout_s"])
    while True:
        check_stop(stop)
        robot.send("observe")
        observation = robot.wait("observation", timeout=5)
        check_stop(stop)
        errors = arrival_errors(observation, target, mask, settings, kinematics, calibration, spec)
        if not errors:
            print("startup: task DATA state arrival confirmed", flush=True)
            return observation
        if time.monotonic() >= deadline:
            raise RuntimeError("startup arrival verification failed: " + str(errors))
        time.sleep(.02)
