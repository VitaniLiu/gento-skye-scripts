#!/usr/bin/env python3
import socket,json,hmac,hashlib,time,pathlib,secrets,math,threading,signal
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from move.msg import State
ROOT=pathlib.Path(__file__).resolve().parent;KEY=(ROOT/'key').read_bytes().strip()
class Gate:
 def __init__(self):self.released=False;self.hold=None
 def step(self,valid,enabled,axes,buttons,now):
  if not valid or not enabled:self.released=False;self.hold=None;return 0.,0.,'disabled_or_stale'
  if not buttons[4]:self.released=True;self.hold=None;return 0.,0.,'release'
  if not self.released:return 0.,0.,'release_required'
  if self.hold is None:
   if abs(axes[0])>.12 or abs(axes[1])>.12:return 0.,0.,'center_before_enable'
   self.hold=now
  def dz(v):return math.copysign((abs(v)-.12)/.88,v) if abs(v)>.12 else 0.
  forward,yaw=dz(axes[1]),dz(axes[0])
  # Suppress small cross-axis motion during an intentional straight/rotate gesture.
  if abs(forward)<.35*abs(yaw):forward=0.
  elif abs(yaw)<.35*abs(forward):yaw=0.
  return -.40*forward,-.50*yaw,'enabled'
def sign(d):
 b=json.dumps(d,separators=(',',':'),sort_keys=True).encode();return json.dumps({'d':d,'mac':hmac.new(KEY,b,hashlib.sha256).hexdigest()}).encode()
def decode(b):
 e=json.loads(b);d=e['d']
 if not hmac.compare_digest(json.loads(sign(d))['mac'],e['mac']):raise ValueError('auth')
 return d

def main():
 a=Context();b=Context();rclpy.init(context=a,domain_id=0);rclpy.init(context=b,domain_id=29)
 ctrl=rclpy.create_node('host_gamepad_control',context=a);rec=rclpy.create_node('host_gamepad_recording',context=b)
 q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
 out=ctrl.create_publisher(TwistStamped,'/move/ManualMoveCmd',q)
 joy=rec.create_publisher(Joy,'/recording/gamepad/joy',10);cmd=rec.create_publisher(TwistStamped,'/recording/gamepad/command',10);status=rec.create_publisher(String,'/recording/gamepad/status',10)
 state={'time':0.,'manual':False,'speed':0.}
 def cb(m):state.update(time=time.monotonic(),manual=not m.is_soft_auto,speed=m.speed.linear.x)
 sub=ctrl.create_subscription(State,'/move/State',cb,10)
 ex=SingleThreadedExecutor(context=a);ex.add_node(ctrl);threading.Thread(target=ex.spin,daemon=True).start()
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('6.6.7.100',18761));s.setblocking(False)
 pending={};last=None;lasttime=0.;lastpoll=-1.;seq=-1;gate=Gate();was_active=False;ticks=0
 stop=threading.Event()
 signal.signal(signal.SIGTERM,lambda *_:stop.set())
 signal.signal(signal.SIGINT,lambda *_:stop.set())
 try:
  while rclpy.ok(context=b) and not stop.is_set():
   start=time.monotonic();nonce=secrets.token_hex(8);pending[nonce]=start
   pending={k:v for k,v in pending.items() if start-v<.15}
   s.sendto(sign({'kind':'poll','nonce':nonce}),('6.6.7.116',18762))
   for _ in range(64):
    try:
     raw,addr=s.recvfrom(8192)
    except BlockingIOError:break
    try:
     if addr[0]!='6.6.7.116':continue
     d=decode(raw);n=d['nonce']
     if n not in pending or pending[n]<=lastpoll:continue
     if len(d['axes'])!=8 or len(d['buttons'])!=13:continue
     if not all(isinstance(x,(float,int)) and math.isfinite(x) and -1<=x<=1 for x in d['axes']):continue
     if not all(x in (0,1) for x in d['buttons']):continue
     lastpoll=pending.pop(n);last=d;seq=d['seq'];lasttime=start
    except (ValueError,KeyError,TypeError):continue
   valid=bool(last and last['connected'] and last.get('report_age_s',999)<.15 and start-lasttime<.15)
   try:armed=start<float((ROOT/'armed_until').read_text())
   except (OSError,ValueError):armed=False
   manual=state['manual'] and start-state['time']<.2
   axes=last['axes'] if valid else [0.]*8;buttons=last['buttons'] if valid else [0]*13
   x,w,reason=gate.step(valid,armed and manual,axes,buttons,start)
   stamp=rec.get_clock().now().to_msg()
   j=Joy();j.header.stamp=stamp;j.header.frame_id='host_gamepad_ds4';j.axes=[float(v) for v in axes];j.buttons=buttons
   if valid:joy.publish(j)
   m=TwistStamped();m.header.stamp=stamp;m.header.frame_id='base_link';m.twist.linear.x=x;m.twist.angular.z=w
   cmd.publish(m)
   active=armed and manual
   if active or was_active:out.publish(m)
   was_active=active
   st={'connected':valid,'armed':armed,'software_manual':manual,'reason':reason,'seq':seq,'host_time_ns':last.get('host_time_ns') if last else None,'usb_report_age_s':last.get('report_age_s') if last else None,'linear_x':x,'angular_z':w,'base_speed_x':state['speed'],'limits':{'linear_m_s':.40,'angular_rad_s':.50,'hold_seconds':None,'usb_timeout_s':.15,'network_timeout_s':.15},'mapping':'axis1 forward(-), axis0 yaw(-); button4 LB/L1 hold'}
   stmsg=String();stmsg.data=json.dumps(st);status.publish(stmsg)
   ticks+=1
   if ticks%50==0:
    (ROOT/'status.json').write_text(json.dumps(st))
   time.sleep(max(0,.02-(time.monotonic()-start)))
 finally:
  if was_active:
   m=TwistStamped()
   for _ in range(5):out.publish(m);time.sleep(.02)
  ex.shutdown();ctrl.destroy_node();rec.destroy_node()
if __name__=='__main__':main()
