"""Split manual motion into measured routes without driving or timing out."""
import json
from pathlib import Path

class RouteRecorder:
    def __init__(self,notify,open_file):
        self.notify=notify;self.open_file=open_file
        self.stream=None;self.path=None;self.number=0
        self.last=None;self.stationary_at=None;self.last_stopped=False

    def write(self,row):
        self.stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        self.stream.flush()

    def finish(self,reason,confirmed=False):
        if self.stream is None:return
        event=dict(type='manual_route_finished',route=self.number,
                   route_log=str(self.path),reason=reason,
                   stationary_confirmed=confirmed,end_pose=self.last.get('pose') if self.last else None,
                   end_robot_monotonic_s=self.last.get('robot_monotonic_s') if self.last else None)
        try:self.write(event)
        finally:self.stream.close();self.stream=None
        self.notify(event)
        self.stationary_at=None

    def observe(self,row):
        kind=row.get('type')
        if kind=='reset_started':
            self.finish('reset_boundary',self.last_stopped)
            return
        if kind!='target' or row.get('phase') not in (None,'PREFLIGHT'):return
        moving=any(row.get(k,0.) for k in ('vx','vy','wz'))
        # Only manual enabled output starts a route. Zero, A-return, and
        # replay targets must never create another manual route.
        if self.stream is None:
            if not moving or row.get('reason')!='enabled':return
            self.path,self.stream=self.open_file();self.number+=1
            self.notify(dict(type='manual_route_started',route=self.number,route_log=str(self.path)))
            self.write(dict(type='manual_route_started',route=self.number,
                            session=row.get('session'),start_pose=row.get('pose'),
                            start_robot_monotonic_s=row.get('robot_monotonic_s')))
        self.last=dict(row);self.write(row)
        p=row.get('pose') or row.get('feedback') or {}
        self.last_stopped=bool(p) and abs(p.get('vx',1.))<=.003 and abs(p.get('vy',1.))<=.003 and abs(p.get('wz',1.))<=.01
        if not moving and self.last_stopped:
            t=row['robot_monotonic_s']
            if self.stationary_at is None:self.stationary_at=t
            if t-self.stationary_at>=.5-1e-9:self.finish('manual_stopped',True)
        else:self.stationary_at=None

    def close(self,reason='session_closed',stop_confirmed=False):
        self.finish(reason,stop_confirmed and self.last_stopped)
