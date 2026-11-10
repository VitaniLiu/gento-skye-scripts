import sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'/'reset'),str(ROOT/'tests'),str(ROOT/'src'/'common'),str(ROOT)]
from project_paths import configure_imports
configure_imports()
from test_hybrid import record,Vehicle
from logic import Sample
routes=[
[(.2,0.,2.68)],
[(.2,0.,2.),(0.,0.,.8),(0.,.2,4.),(0.,0.,.5)],
[(0.,.2,3.),(0.,0.,.8),(.2,0.,1.),(0.,0.,.8),(0.,-.2,3.),(0.,0.,.8),(.2,0.,1.)]]
worst=(0.,0.);count=0
for route in routes:
 for gain in (.85,1.08,1.2):
  for tau in (.08,.15,.3):
   trip,v,t,i=record(route);v.gain=gain;v.tau=tau
   f=trip.controller(None,t)
   for n in range(6000):
    sample=Sample(round(v.x,3),round(v.y,3),math.radians(round(math.degrees(v.a),1)),v.v,0.,v.w,t,i)
    cmd=f.step(sample,t)
    if f.finished_reason:break
    v.step(cmd.vx,cmd.wz);t+=.02;i+=1
   error=math.hypot(v.x,v.y);yaw=abs(math.degrees(v.a))
   assert f.finished_reason=='reset_complete',(gain,tau,f.finished_reason,error,yaw)
   assert error<=.007 and yaw<=.5,(gain,tau,error,yaw)
   worst=(max(worst[0],error),max(worst[1],yaw));count+=1
print(dict(cases=count,worst_position_mm=worst[0]*1000,worst_yaw_deg=worst[1],quantized_odometry=True,motion_started=False))
