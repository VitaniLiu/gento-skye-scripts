#!/usr/bin/env python3
import json,pathlib,time,sys
r=pathlib.Path(__file__).resolve().parent;op=sys.argv[1] if len(sys.argv)>1 else 'status'
if op=='stop':
 (r/'armed_until').unlink(missing_ok=True);print('已关闭运动输出；输入和录制服务继续运行。')
elif op=='start':
 s=json.loads((r/'status.json').read_text())
 if time.time()-(r/'status.json').stat().st_mtime>3 or not s['connected'] or not s['software_manual']:
  sys.exit('未启用：请确认手柄连接、接收服务运行且底盘处于软件手动模式。')
 (r/'armed_until').write_text(str(time.monotonic()+1800))
 print('已启用30分钟：左摇杆回中，按住LB/L1后操作；松键停车，可持续按住。上限0.40m/s、0.50rad/s。')
else:print((r/'status.json').read_text())
