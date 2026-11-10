"""Read-only robot-local MCAP timeline. No ROS publishers are created."""
import json
from pathlib import Path

def load(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    root=Path(path).resolve()
    manifest=json.loads((root/'recording.json').read_text())
    if manifest.get('file_status')!='closed':
        raise ValueError('Recording is not closed')
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'domain_24'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    rows=[]
    while reader.has_next():
        topic,data,stamp=reader.read_next()
        rows.append((int(stamp),topic,data))
    if not rows:raise ValueError('Empty recording')
    origin=min(r[0] for r in rows)
    streams={}
    for stamp,topic,data in rows:
        streams.setdefault(topic,[]).append((stamp-origin,deserialize_message(data,types[topic])))
    for stream in streams.values():stream.sort(key=lambda row:row[0])
    return dict(origin_ns=origin,duration_ns=max(r[0] for r in rows)-origin,streams=streams,
                quality_status=manifest.get('quality_status'),source_clock_mapping=manifest.get('source_clock_mapping'))

def summary(plan):
    return dict(origin_ns=plan['origin_ns'],duration_ns=plan['duration_ns'],
                quality_status=plan['quality_status'],source_clock_mapping=plan['source_clock_mapping'],
                streams={k:dict(count=len(v),first_offset_ns=v[0][0],last_offset_ns=v[-1][0]) for k,v in plan['streams'].items()})
