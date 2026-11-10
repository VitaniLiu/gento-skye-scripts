"""Bounded capture metadata mailbox; never publishes velocity or opens SSH."""
import json,os,re,tempfile,time,uuid
from pathlib import Path

def atomic_json(path,value):
 path=Path(path)
 if path.is_symlink():raise ValueError("capture mailbox symlink forbidden")
 fd,tmp=tempfile.mkstemp(prefix=".capture-",dir=path.parent)
 try:
  with os.fdopen(fd,"w") as f:
   os.fchmod(f.fileno(),0o600);json.dump(value,f,allow_nan=False);f.write("\n");f.flush();os.fsync(f.fileno())
  os.replace(tmp,path)
 finally:
  if os.path.exists(tmp):os.unlink(tmp)

def valid_request(row):
 return (isinstance(row,dict) and row.get("type")=="capture" and
         row.get("operation") in ("begin","end","abort") and
         all(re.fullmatch("[0-9a-f]{32}",str(row.get(k,""))) for k in ("session","request_id","capture_id")))

class CaptureMailbox:
 def __init__(self,log,session):
  self.log=Path(log);self.session=session;self.seen=set()
  self.descriptor=self.log.with_suffix(".capture-control.json")
  self.request_path=self.log.with_suffix(".capture-request.json")
  atomic_json(self.descriptor,dict(schema=1,session=session,active=True,request_path=str(self.request_path)))
 def receive(self):
  if not self.request_path.exists():return []
  if self.request_path.stat().st_size>4096:raise ValueError("capture request too large")
  row=json.loads(self.request_path.read_text())
  if not valid_request(row):raise ValueError("invalid capture request")
  if row["session"]!=self.session or row["request_id"] in self.seen:return []
  deadline=row.get("deadline_host_ns")
  if type(deadline) is not int or not 0<=deadline-time.time_ns()<=10_000_000_000:return []
  self.seen.add(row["request_id"])
  if len(self.seen)>64:self.seen={row["request_id"]}
  return [row]
 def close(self):
  atomic_json(self.descriptor,dict(schema=1,session=self.session,active=False,request_path=str(self.request_path)))

class CaptureClient:
 def __init__(self,log,timeout=3.):
  self.log=Path(log);self.timeout=timeout
 def request(self,operation,capture_id):
  descriptor=json.loads(self.log.with_suffix(".capture-control.json").read_text())
  if descriptor.get("schema")!=1 or descriptor.get("active") is not True:
   raise ValueError("底盘采集边界入口未运行，请使用更新后的 gento 底盘遥操入口")
  path=self.log.with_suffix(".capture-request.json")
  if descriptor.get("request_path")!=str(path):raise ValueError("capture mailbox path mismatch")
  request=dict(type="capture",session=descriptor["session"],request_id=uuid.uuid4().hex,
               capture_id=capture_id,operation=operation,deadline_host_ns=time.time_ns()+int(self.timeout*1e9))
  if not valid_request(request):raise ValueError("invalid capture identity or operation")
  with self.log.open() as stream:
   stream.seek(0,2);identity=(self.log.stat().st_dev,self.log.stat().st_ino);offset=stream.tell()
   atomic_json(path,request);deadline=time.monotonic()+self.timeout
   try:
    while time.monotonic()<deadline:
     st=self.log.stat()
     if (st.st_dev,st.st_ino)!=identity or st.st_size<offset:raise ValueError("sender log replaced")
     pos=stream.tell();line=stream.readline()
     if not line.endswith("\n"):
      stream.seek(pos);time.sleep(.005);continue
     try:row=json.loads(line)
     except ValueError:continue
     if row.get("type")!="capture_boundary" or row.get("request_id")!=request["request_id"]:continue
     if any(row.get(k)!=request[k] for k in ("session","capture_id","operation")):raise ValueError("capture acknowledgement identity mismatch")
     if row.get("ok") is not True:raise ValueError(row.get("reason","capture boundary rejected"))
     return row
    raise TimeoutError("底盘未确认采集边界；本次没有开始采集")
   finally:
    if path.exists():
     try:
      if json.loads(path.read_text()).get("request_id")==request["request_id"]:path.unlink()
     except (ValueError,OSError):pass
