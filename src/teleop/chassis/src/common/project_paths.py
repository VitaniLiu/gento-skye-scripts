"""Project locations and flat-module compatibility after source separation."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
MODULE_DIRS=tuple(ROOT/"src"/name for name in ("common","teleop","reset","record","robot"))
def configure_imports():
    for p in reversed(MODULE_DIRS):
        if str(p) not in sys.path:sys.path.insert(0,str(p))
    return ROOT


def output_directory(kind, root=ROOT):
    """Resolve new outputs separately from immutable imported history; no I/O on import."""
    import json, os
    root=Path(root).resolve()
    if kind not in ('logs', 'routes'): raise ValueError('Unknown output kind')
    integrated=root.name=='chassis' and root.parent.name=='teleop' and root.parent.parent.name=='src'
    project=root.parents[2] if integrated else root
    config=project/'configs/teleop/chassis.json'
    settings=json.loads(config.read_text()) if integrated and config.exists() else {}
    default=('logs/teleop' if kind=='logs' else 'logs/teleop/routes')
    value=os.environ.get('CHASSIS_'+kind.upper()+'_DIR',settings.get(kind+'_dir',default))
    path=Path(value).expanduser()
    path=(path if path.is_absolute() else project/path).resolve()
    forbidden=[Path('/home/descfly/tianjidipan'),root/'reference',root/'data',root/'logs'] if integrated else [root/'reference',root/'data/routes',root/'logs/sessions',root/'logs/preview',root/'logs/transport']
    if any(path==p.resolve() or path.is_relative_to(p.resolve()) for p in forbidden):
        raise ValueError('Output would overwrite source or historical material')
    return path
