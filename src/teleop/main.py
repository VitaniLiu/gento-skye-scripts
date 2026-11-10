"""Gento Skye coordinator; default/help/import are strictly offline."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from teleop.control import (ROOT, ControlGate, ChassisAdapter, JsonProcess,
                           native_command, native_problems, NativeObserver)


def configuration():
    common = json.loads((ROOT/'configs/common/runtime.json').read_text())
    config = {**common, **json.loads((ROOT/'configs/teleop/runtime.json').read_text())}
    native_command(config)  # Build/validate only: does not open a connection.
    for key in ('feedback_timeout_s', 'observer_timeout_s', 'startup_timeout_s', 'vr_timeout_s'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid timeout: '+key)
    if config['observer_timeout_s'] > 1:
        raise ValueError('observer_timeout_s must be <= 1')
    if config.get('motion_enabled') is not False:
        raise ValueError('default motion_enabled must remain false; use explicit --execute')
    for key in ('device', 'hidraw', 'chassis_remote_dir'):
        if not isinstance(config[key], str) or not config[key].startswith('/'):
            raise ValueError('absolute path required: '+key)
    # Validate existing sender arguments without importing or invoking its transport.
    if not (config['chassis_remote_dir'].startswith('/home/user/tianjidipan') or config['chassis_remote_dir']=='/home/user/gento-skye_scripts/src/teleop/chassis'):
        raise ValueError('unsupported robot chassis deployment')
    chassis = json.loads((ROOT/'configs/teleop/chassis.json').read_text())
    for key in ('logs_dir', 'routes_dir'):
        path = (ROOT/chassis[key]).resolve()
        if not path.is_relative_to((ROOT/'logs/teleop').resolve()):
            raise ValueError('production chassis outputs must remain under logs/teleop')
    return config


def emit(row):
    print(json.dumps(row, ensure_ascii=False, allow_nan=False), flush=True)


class SessionLog:
    def __init__(self, output=None):
        self.session = uuid.uuid4().hex
        self.folder = ROOT/'logs/teleop' if output is None else Path(output).resolve()
        if output is not None and not self.folder.is_relative_to((ROOT/'tests').resolve()):
            raise ValueError('--test-output must be inside project tests/')
        self.folder.mkdir(parents=True, exist_ok=True)
        self.file = (self.folder/('coordinator-'+self.session+'.jsonl')).open('x')
        self.errors = (self.folder/('coordinator-'+self.session+'.stderr')).open('x')
        self.env = os.environ.copy()
        self.env['PYTHONDONTWRITEBYTECODE'] = '1'
        if output is not None:
            self.env['CHASSIS_LOGS_DIR'] = str(self.folder)
            self.env['CHASSIS_ROUTES_DIR'] = str(self.folder/'routes')

    def record(self, row):
        row = dict(row, coordinator_session=self.session,
                   host_time=datetime.now(timezone.utc).isoformat())
        self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        self.file.flush()

    def close(self):
        self.errors.close()
        self.file.close()


def wait_native(observer, config):
    deadline = time.monotonic()+config['startup_timeout_s']
    latest = {}
    last_sequence = 0
    while time.monotonic()<deadline:
        for row in observer.receive():
            if row.get('type') != 'native_status' or type(row.get('sequence')) is not int or row['sequence'] <= last_sequence:
                raise ValueError('invalid native observer sequence')
            latest = row
            last_sequence = row['sequence']
        if latest and not native_problems(latest, config):
            return latest
        if observer.eof or observer.process.poll() is not None:
            raise RuntimeError('native observer exited; inspect session stderr')
        time.sleep(.02)
    return latest


def check(config, log):
    observer = NativeObserver(config, log.errors, log.env)
    try:
        row = wait_native(observer, config)
        problems = native_problems(row, config)
        report = dict(type='native_check', problems=problems, snapshot=row,
                      motion_sent=False, upper_stop='manual_physical_estop')
        log.record(report)
        emit(report)
    finally:
        observer.close()
    # Reuse the existing read-only chassis preflight, including endpoint identity.
    adapter = ChassisAdapter(config, log.errors, 20, log.env)
    child = JsonProcess(adapter.command('check'), log.errors, log.env)
    checked = False
    chassis_problems = []
    deadline = time.monotonic()+25
    try:
        while time.monotonic()<deadline:
            for row in child.receive():
                log.record(dict(component='chassis', **row))
                if row.get('type') in ('failed', 'error'):
                    chassis_problems.append(row.get('reason', row.get('error', 'chassis_check_failed')))
                if row.get('type') == 'checked' and row.get('ok') is True:
                    checked = True
            if child.eof:break
            time.sleep(.02)
        try:child_code = child.process.wait(timeout=2)
        except subprocess.TimeoutExpired:child_code = None
        result = dict(type='check_result', native_ready=not problems,
                      chassis_ready=checked and child_code==0,
                      native_problems=problems, chassis_problems=chassis_problems,
                      motion_sent=False, upper_stop='manual_physical_estop')
        log.record(result)
        emit(result)
        return 0 if result['native_ready'] and result['chassis_ready'] else 1
    finally:
        child.close()


def execute(config, log, seconds):
    gate = ControlGate()
    adapter = ChassisAdapter(config, log.errors, seconds, log.env)
    observer = None
    lock = None
    exit_code = 1
    reason = 'startup_failed'
    previous_handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    try:
        # This lock excludes other integrated sessions only. Chassis retains its
        # existing robot lock/ownership checks; Apex remains externally owned.
        lock = (ROOT/'logs/teleop/coordinator.lock').open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[sig] = signal.signal(sig, interrupted)
        gate.acquire('teleop')
        observer = NativeObserver(config, log.errors, log.env)
        snapshot = wait_native(observer, config)
        problems = native_problems(snapshot, config)
        if problems:raise RuntimeError('; '.join(problems))
        identity = {m['id']:m.get('exec_main_pid') for m in snapshot['backend']['modules'] if m['id'] in ('robot','teleop','tool')}
        sequence = snapshot['sequence']
        gate.connect()
        gate.adapter = adapter
        gate.arm(config['observer_timeout_s'])
        log.record(dict(type='native_ready', snapshot=snapshot))
        child = gate.execute()
        started = time.monotonic()
        emit(dict(type='session_started', session=log.session,
                  upper_control='existing_apex_services', upper_stop='manual_physical_estop',
                  chassis_stop='existing_sender_stop_handshake', max_seconds=seconds))
        while time.monotonic()-started < seconds:
            latest = None
            for row in observer.receive():
                if row.get('type') != 'native_status' or type(row.get('sequence')) is not int or row['sequence'] <= sequence:
                    raise RuntimeError('native observer sequence invalid')
                sequence = row['sequence']
                latest = row
            if latest is not None:
                problems = native_problems(latest, config)
                current = {m['id']:m.get('exec_main_pid') for m in latest['backend']['modules'] if m['id'] in identity}
                if current != identity:problems.append('native service restarted')
                if problems:raise RuntimeError('; '.join(problems))
                gate.arm(config['observer_timeout_s'])
            if observer.eof or not gate.check():raise RuntimeError('native observer lost or timed out')
            for row in child.receive():
                adapter.observe(row)
                log.record(dict(component='chassis', **row))
                if row.get('type') in ('ready','reset_started','reset_finished','stopped','final','error'):
                    emit(dict(component='chassis', **row))
            if child.eof:
                reason = 'chassis_session_ended'
                exit_code = 0 if adapter.stop_confirmed and child.process.wait(timeout=2)==0 else 1
                break
            time.sleep(.01)
        else:
            reason = 'session_time_limit'
            exit_code = 0
    except KeyboardInterrupt:
        reason = 'operator_interrupt'
        exit_code = 130
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        reason = str(exc)
    finally:
        # Further signals cannot interrupt the existing bounded chassis stop.
        for sig in previous_handlers:signal.signal(sig, signal.SIG_IGN)
        gate.disconnect()
        stop = {'started':adapter.child is not None, 'stop_confirmed':False}
        try:
            stop = adapter.stop(log.record)
        except Exception as exc:
            reason += '; chassis stop error: '+str(exc)
        finally:
            try:
                if observer is not None:observer.close()
            finally:
                gate.release()
                if lock is not None:lock.close()
                for sig, handler in previous_handlers.items():signal.signal(sig, handler)
        if stop['started'] and stop['stop_confirmed'] is not True:exit_code = 2
        result = dict(type='coordinator_final', reason=reason, chassis=stop,
                      upper_stop_confirmed=None, upper_services='left_running',
                      upper_stop='manual_physical_estop', exit_code=exit_code)
        log.record(result)
        emit(result)
    return exit_code


def main(feature='teleop', argv=None):
    parser = argparse.ArgumentParser(description='Pico/Apex + existing chassis coordination; default offline, native E-stop is manual')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Explicit robot read-only preflight')
    mode.add_argument('--execute', action='store_true', help='Real chassis control while observing existing Apex Pico/gripper services')
    mode.add_argument('--preview', action='store_true', help='Existing synthetic chassis preview; no hardware or logging')
    parser.add_argument('--seconds', type=float, default=60, help='Bounded integrated session duration (default 60s)')
    parser.add_argument('--test-output', help='Redirect host inspection/test outputs into project tests/')
    parser.add_argument('--init-storage', action='store_true', help='Retained explicit foundation storage initialization')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        parser.error('--seconds must be finite and within (0, 3600]')
    if args.execute and args.test_output:parser.error('--test-output is for non-motion checks only')
    if args.init_storage and (args.check or args.execute or args.preview):parser.error('--init-storage is offline-only')
    try:
        config = configuration()
        if args.init_storage:
            from processing.runtime import initialize_storage
            initialize_storage(config)
        if args.preview:
            return subprocess.call([sys.executable, '-B', str(ROOT/'src/teleop/chassis/src/teleop/preview.py'), '--demo', '--no-log'])
        if not args.check and not args.execute:
            emit(dict(feature=feature, configuration='valid', motion_enabled=False,
                      native_services='externally_managed', upper_stop='manual_physical_estop',
                      ros_domain_id=config['ros_domain_id'],
                      chassis_sender='src/teleop/chassis/src/teleop/sender.py'))
            return 0
        log = SessionLog(args.test_output)
        try:
            return execute(config, log, args.seconds) if args.execute else check(config, log)
        finally:
            log.close()
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        emit(dict(type='error', error=str(exc), upper_stop='manual_physical_estop'))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
