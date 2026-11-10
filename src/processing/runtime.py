"""Offline foundation. Contains no robot transport or SDK calls."""
import json, math, os, re, uuid, time, inspect
from pathlib import Path
from datetime import datetime, timezone
ROOT = Path(__file__).resolve().parents[2]
FEATURES = ('teleop','recording','processing','replay')
STAGES = ('raw_data','validations','conversions','lerobot','replays')

def fail(reason, **context):
    caller = inspect.currentframe().f_back
    raise ValueError(json.dumps(dict(severity='FAULT', file=str(Path(caller.f_code.co_filename).resolve()), function=caller.f_code.co_name, reason=reason, context=context), ensure_ascii=False))

def read_json(path):
    path=Path(path).resolve()
    try: return json.loads(path.read_text())
    except (OSError, ValueError) as exc: fail('cannot read JSON', path=str(path), detail=str(exc))

def load_config(feature):
    if feature not in FEATURES: fail('unknown feature', feature=feature)
    common=read_json(ROOT/'configs/common/runtime.json')
    config={**common, **read_json(ROOT/f'configs/{feature}/runtime.json')}
    if config.get('motion_enabled') is not False: fail('motion unavailable in foundation release')
    for key in ('storage_mount','project_root'):
        if not Path(config[key]).is_absolute(): fail('absolute path required', key=key)
    raw=config.get('robot_raw_root')
    if raw is not None and not Path(raw).is_absolute(): fail('robot raw root must be absolute')
    return config

def new_id(kind):
    if kind not in STAGES+('recording','validation','conversion','replay','run'): fail('unknown ID kind', kind=kind)
    return kind+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_'+uuid.uuid4().hex

def exclusive_json(path, value):
    path=Path(path)
    try:
        with path.open('x') as f:
            json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False); f.write('\n'); f.flush(); os.fsync(f.fileno())
    except (OSError,ValueError) as exc: fail('exclusive JSON write failed', path=str(path.resolve()), detail=str(exc))

def data_root(config):
    mount=Path(config['storage_mount']).resolve()
    if not mount.is_mount(): fail('data disk is not mounted', path=str(mount))
    root=mount/'gento-skye_data'
    if root.is_symlink(): fail('data root symlink forbidden', path=str(root))
    return root

def initialize_storage(config):
    root=data_root(config)
    try:
        for part in STAGES+('temporary/transfers','temporary/conversions','temporary/replays'):
            target=root/part
            if target.is_symlink(): fail('storage symlink forbidden', path=str(target))
            target.mkdir(parents=True,exist_ok=True)
    except OSError as exc: fail('cannot initialize external storage', path=str(root), detail=str(exc))
    return root

def log(feature, severity, reason, **context):
    if feature not in FEATURES or severity not in ('INFO','WARNING','FAULT'): fail('invalid log category')
    caller=inspect.currentframe().f_back
    # Do not accept authentication fields, including nested objects.
    def scrub(v):
        if isinstance(v,dict): return {k:('[REDACTED]' if re.search('password|passwd|secret|token|credential',k,re.I) else scrub(x)) for k,x in v.items()}
        if isinstance(v,list): return [scrub(x) for x in v]
        return v
    record=dict(time_utc=datetime.now(timezone.utc).isoformat(),severity=severity,file=str(Path(caller.f_code.co_filename).resolve()),function=caller.f_code.co_name,reason=reason,context=scrub(context))
    path=ROOT/'logs'/feature/'runtime.jsonl'
    with path.open('a') as f: f.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
    return path
