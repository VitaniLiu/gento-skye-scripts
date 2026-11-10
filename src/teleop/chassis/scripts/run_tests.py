"""Redirect imported tests to the integrated isolated test runner."""
from pathlib import Path
import runpy
root=Path(__file__).resolve().parents[1]
runner=root.parents[2]/'tests/chassis_migration/run_offline.py'
if not runner.is_file():raise SystemExit('Use the integrated project tests/chassis_migration/run_offline.py')
runpy.run_path(str(runner),run_name='__main__')
