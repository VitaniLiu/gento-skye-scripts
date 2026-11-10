#!/usr/bin/env python3
import os,socket,select,struct,json,hmac,hashlib,time,pathlib
ROOT=pathlib.Path(__file__).resolve().parent
KEY=(ROOT/'key').read_bytes().strip()
DEV='/dev/input/by-id/usb-Sony_Interactive_Entertainment_Wireless_Controller-joystick'
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('6.6.7.116',18762));s.setblocking(False)
h=None;last_report=0.;last_packet=None;report_count=0
f=None;axes=[0.0]*8;buttons=[0]*13;connected=False;seq=0;next_open=0

def sign(d):
 b=json.dumps(d,separators=(',',':'),sort_keys=True).encode();return json.dumps({'d':d,'mac':hmac.new(KEY,b,hashlib.sha256).hexdigest()}).encode()
def decode(b):
 e=json.loads(b);d=e['d'];expected=json.loads(sign(d))['mac']
 if not hmac.compare_digest(expected,e['mac']):raise ValueError('auth')
 return d
while True:
 now=time.monotonic()
 if h is None:
  try:h=os.open('/dev/gamepad-control-hid',os.O_RDONLY|os.O_NONBLOCK);last_report=0.;last_packet=None
  except OSError:pass
 if f is None and now>=next_open:
  next_open=now+1
  try:
   f=os.open(DEV,os.O_RDONLY|os.O_NONBLOCK);axes=[0.0]*8;buttons=[0]*13;connected=True
   print('controller connected',flush=True)
  except OSError:connected=False
 ready=select.select([s]+([f] if f is not None else [])+([h] if h is not None else []),[],[],.02)[0]
 if h is not None and h in ready:
  try:
   for _ in range(512):
    try:packet=os.read(h,128)
    except BlockingIOError:break
    if len(packet)!=64 or packet[0]!=1:raise OSError('invalid DS4 report')
    if packet!=last_packet:last_report=time.monotonic();last_packet=packet;report_count+=1
  except OSError:os.close(h);h=None;last_report=0.
 if f is not None and (f in ready or not os.path.exists(DEV)):
  try:
   if not os.path.exists(DEV):raise OSError('removed')
   for _ in range(512):
    try:b=os.read(f,8)
    except BlockingIOError:break
    if len(b)!=8:raise OSError('short read')
    stamp,value,typ,num=struct.unpack('IhBB',b);typ&=127
    if typ==1 and num<13:buttons[num]=int(bool(value))
    if typ==2 and num<8:axes[num]=max(-1.,min(1.,value/32767.))
  except OSError:
   os.close(f);f=None;connected=False;buttons=[0]*13;axes=[0.0]*8
   print('controller disconnected',flush=True)
 if s in ready:
  try:
   b,addr=s.recvfrom(8192)
   if addr[0]!='6.6.7.100':continue
   d=decode(b)
   if d.get('kind')!='poll':continue
   seq+=1
   s.sendto(sign({'kind':'input','nonce':d['nonce'],'seq':seq,'connected':connected and time.monotonic()-last_report<.15,'report_age_s':time.monotonic()-last_report,'report_count':report_count,'axes':axes,'buttons':buttons,'host_time_ns':time.time_ns(),'device':'DS4-compatible 054c:09cc'}),addr)
  except (ValueError,KeyError,TypeError,OSError):pass
