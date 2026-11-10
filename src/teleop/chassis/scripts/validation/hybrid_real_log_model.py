import json,pathlib,sys,math
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'/'reset'),str(ROOT/'tests'),str(ROOT/'src'/'common'),str(ROOT)]
from hybrid import RecordedTrip
from logic import Sample
from project_paths import configure_imports
configure_imports()
from test_hybrid import Vehicle
names=['live-20261102-122117-de3b2321.jsonl','live-20261102-122357-a2d3e94c.jsonl','live-20261102-125634-be244484.jsonl','live-20261102-141232-2d793082.jsonl']
for name in names:
 trip=RecordedTrip(None)
 rows=[json.loads(l) for l in ((ROOT/'logs'/'sessions'/'2026-11-02'/name) if (ROOT/'logs'/'sessions'/'2026-11-02'/name).exists() else (ROOT/'logs'/'sessions'/'2026-11-02'/name)).read_text().splitlines()]
 last=None
 for r in rows:
  if r.get('type')=='reset_started' or r.get('phase')=='RETRACE':break
  if r.get('type')!='target' or not r.get('pose'):continue
  q=r['pose'];s=Sample(*(q[k] for k in ('x','y','yaw','vx','vy','wz','at','seq')),q.get('observed_at'))
  trip.observe(s,r['robot_monotonic_s'],r['wz'],commanded_vx=r['vx']);last=q
 assert trip.fault is None,(name,trip.fault)
 for gain in (.85,1.08,1.2):
  v=Vehicle(gain);v.x=last['x'];v.y=last['y'];v.a=last['yaw']
  f=trip.controller(None,0.)
  for i in range(6000):
   t=i*.02
   c=f.step(v.sample(t,i),t)
   if f.finished_reason:break
   v.step(c.vx,c.wz)
  err=math.hypot(v.x-trip.origin.x,v.y-trip.origin.y)
  yaw=math.degrees(math.atan2(math.sin(v.a-trip.origin.yaw),math.cos(v.a-trip.origin.yaw)))
  print(json.dumps(dict(log=name,gain=gain,status=f.finished_reason,error_mm=err*1000,yaw_deg=yaw,seconds=t,leg=f.index)))
