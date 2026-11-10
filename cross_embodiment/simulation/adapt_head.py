"""Map source head-relative rotation to Skye yaw/pitch and clamp to URDF limits."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def rotation6d(values):
    x = np.asarray(values[:3], dtype=float)
    y = np.asarray(values[3:6], dtype=float)
    x /= np.linalg.norm(x)
    y -= x * np.dot(x, y)
    y /= np.linalg.norm(y)
    return np.column_stack((x, y, np.cross(x, y)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    project = Path(__file__).resolve().parents[1]
    allowed = project.parent / "tests/cross_embodiment"
    if not output.is_relative_to(allowed) or output.exists():
        raise ValueError("Choose a new output directory under tests/cross_embodiment")

    model_path = project / "models/skye/scene.xml"
    urdf_path = project / "models/skye/reference.urdf"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key)
    data.qpos[model.joint("waist_joint_1").qposadr[0]] = 0.49631398916244507
    head_names = ["Head_J1_Joint", "Head_J2_Joint"]
    head_ids = np.array([model.joint(name).qposadr[0] for name in head_names])
    head_body = model.body("eyes_link").id
    urdf = ET.parse(urdf_path).getroot()
    limits = {
        joint.get("name"): [float(joint.find("limit").get(key)) for key in ("lower", "upper", "velocity")]
        for joint in urdf.findall("joint") if joint.find("limit") is not None
    }
    low, high, speed = np.array([limits[name] for name in head_names]).T
    neutral_q = data.qpos[head_ids].copy()
    mujoco.mj_forward(model, data)
    neutral_rotation = data.xmat[head_body].reshape(3, 3).copy()

    numeric = pq.read_table(source, columns=["action", "observation.state", "timestamp"])
    timestamps = np.asarray(numeric["timestamp"].to_pylist(), dtype=float)
    adapted = {}
    reports = {}
    for field in ("action", "observation.state"):
        values = np.asarray(numeric[field].to_pylist(), dtype=float)
        source_rotations = [rotation6d(row[21:27]) for row in values]
        source_initial = source_rotations[0]
        previous_unconstrained = neutral_q.copy()
        previous_clamped = neutral_q.copy()
        generated = []
        unconstrained_rows = []
        clamped_rows = []
        residual_degrees = []
        limit_clips = np.zeros(2, dtype=int)
        rate_clips = np.zeros(2, dtype=int)
        for index, source_rotation in enumerate(source_rotations):
            desired = source_rotation @ source_initial.T @ neutral_rotation

            def residual(q):
                data.qpos[head_ids] = q
                mujoco.mj_forward(model, data)
                actual = data.xmat[head_body].reshape(3, 3)
                return Rotation.from_matrix(desired @ actual.T).as_rotvec()

            fit = least_squares(
                residual, previous_unconstrained, bounds=(-np.pi, np.pi),
                max_nfev=100, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            unconstrained = fit.x
            clamped = np.clip(unconstrained, low, high)
            limit_clips += ~np.isclose(clamped, unconstrained, atol=1e-9)
            if index:
                step = speed * (timestamps[index] - timestamps[index - 1])
                rate_limited = np.clip(clamped, previous_clamped - step, previous_clamped + step)
                rate_clips += ~np.isclose(rate_limited, clamped, atol=1e-9)
                clamped = rate_limited
            data.qpos[head_ids] = clamped
            mujoco.mj_forward(model, data)
            actual = data.xmat[head_body].reshape(3, 3).copy()
            position = data.xpos[head_body].copy()
            generated.append(np.r_[position, actual[:, 0], actual[:, 1]])
            residual_degrees.append(float(Rotation.from_matrix(desired @ actual.T).magnitude() * 180 / np.pi))
            unconstrained_rows.append(unconstrained.copy())
            clamped_rows.append(clamped.copy())
            previous_unconstrained = unconstrained
            previous_clamped = clamped
        values[:, 18:27] = np.asarray(generated)
        adapted[field] = values
        unconstrained_rows = np.asarray(unconstrained_rows)
        clamped_rows = np.asarray(clamped_rows)
        reports[field] = {
            "head_joint_names": head_names,
            "urdf_limits_rad": np.column_stack((low, high)).tolist(),
            "velocity_limits_rad_s": speed.tolist(),
            "neutral_q_rad": neutral_q.tolist(),
            "limit_clipped_frames_per_joint": limit_clips.tolist(),
            "rate_clipped_frames_per_joint": rate_clips.tolist(),
            "unconstrained_q_min_rad": unconstrained_rows.min(axis=0).tolist(),
            "unconstrained_q_max_rad": unconstrained_rows.max(axis=0).tolist(),
            "adapted_q_min_rad": clamped_rows.min(axis=0).tolist(),
            "adapted_q_max_rad": clamped_rows.max(axis=0).tolist(),
            "max_orientation_projection_error_deg": max(residual_degrees),
            "mean_orientation_projection_error_deg": float(np.mean(residual_degrees)),
        }

    (output / "data/chunk-000").mkdir(parents=True)
    (output / "meta").mkdir()
    destination = output / "data/chunk-000" / source.name
    writer = None
    offset = 0
    for batch in pq.ParquetFile(source).iter_batches(batch_size=8):
        table = pa.Table.from_batches([batch])
        length = len(table)
        for field in ("action", "observation.state"):
            column = table.schema.get_field_index(field)
            table = table.set_column(
                column, table.schema.field(column),
                pa.array(adapted[field][offset:offset + length].tolist(), type=table.schema.field(column).type))
        if writer is None:
            writer = pq.ParquetWriter(destination, table.schema, compression="zstd")
        writer.write_table(table)
        offset += length
    writer.close()

    for name in ("frame_quality.parquet", "schema78.json"):
        shutil.copy2(source.parents[2] / "meta" / name, output / "meta" / name)
    export = json.loads((source.parents[2] / "meta/export.json").read_text())
    export["policy"]["ik_position_tolerance_m"] = 0.02
    export["policy"]["ik_rotation_tolerance_rad"] = float(np.deg2rad(10.0))
    export["head_adaptation"] = {
        "method": "source-relative orientation mapped to Skye neutral orientation; unconstrained yaw/pitch projected then angle- and rate-clamped to URDF limits; output regenerated by Skye FK",
        "source_first_frame_is_neutral_reference": True,
        "source_position_ignored": True,
        "source_roll_not_directly_representable": True,
    }
    (output / "meta/export.json").write_text(json.dumps(export, indent=2))
    report = {
        "frames": len(timestamps),
        "source": str(source),
        "source_sha256": digest(source),
        "output_sha256": digest(destination),
        "only_pose_values_changed": ["action[18:27]", "observation.state[18:27]"],
        "position_tolerance_m": 0.02,
        "rotation_tolerance_deg": 10.0,
        "fields": reports,
        "source_unchanged": True,
        "no_motion": True,
    }
    (output / "head-adaptation-report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
