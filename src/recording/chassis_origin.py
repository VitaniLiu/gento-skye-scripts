"""Capture-boundary metadata client; starts no process and sends no velocity."""
import importlib.util,json,uuid,math
from pathlib import Path
_path=Path(__file__).resolve().parents[1]/"teleop/chassis/src/common/capture_control.py"
_spec=importlib.util.spec_from_file_location("gento_capture_metadata",_path)
_protocol=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_protocol)

def validate_origin(value):
 if not isinstance(value,dict) or value.get("ok") is not True or value.get("operation")!="begin":
  raise ValueError("invalid chassis capture origin")
 import re
 if not re.fullmatch("[0-9a-f]{32}",str(value.get("capture_id",""))):raise ValueError("invalid capture id")
 origin=value.get("origin",{})
 if not all(type(origin.get(k)) in (int,float) and math.isfinite(origin[k]) for k in ("x","y","yaw","at")):
  raise ValueError("invalid chassis origin pose")
 return value

class ChassisCapture:
 def __init__(self,source):
  self.client=_protocol.CaptureClient(source);self.capture_id=uuid.uuid4().hex
  self.active=False;self.ended=False;self.stop_requested=False
  self.recording_ready=False;self.cleanup_pending=False
 def _request(self,operation):
  for attempt in range(2):
   try:return self.client.request(operation,self.capture_id)
   except TimeoutError:
    if attempt:raise
 def begin(self):
  self.active=True
  try:return validate_origin(self._request("begin"))
  except BaseException:
   try:self.abort()
   except Exception as exc:
    self.cleanup_pending=True
    raise RuntimeError("底盘开始状态未确认，未启动采集；请检查遥操连接后重试") from exc
   raise
 def end(self):
  if self.active and not self.ended:
   self.stop_requested=True
   ack=self._request("end");self.ended=True;return ack
 def abort(self,force=False):
  if self.active and (force or not self.ended):
   self.cleanup_pending=True
   ack=self._request("abort");self.ended=True;self.cleanup_pending=False;return ack
