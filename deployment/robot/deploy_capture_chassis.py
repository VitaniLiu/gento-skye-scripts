"""Deploy this Gento capture-boundary runtime only; never start controllers."""
from pathlib import Path
import argparse,hashlib,json,subprocess,shlex
ROOT=Path(__file__).resolve().parents[2]
CHASSIS=ROOT/'src/teleop/chassis'
REMOTE=r"""
from pathlib import Path
import sys,json,hashlib,os,fcntl,tempfile,time
p=json.load(sys.stdin);base=Path('/home/user/gento-skye_scripts')
if base.is_symlink() or base.resolve()!=base:raise ValueError('unsafe Gento root')
root=base/'src/teleop/chassis'
def safe(rel):
 path=base/rel
 if Path(rel).is_absolute() or '..' in Path(rel).parts or path.is_symlink() or not path.resolve().is_relative_to(base):raise ValueError('unsafe path')
 return path
for rel,text in p['files'].items():
 path=safe(rel)
 if path.exists() and path.read_text()!=text:
  actual=hashlib.sha256(path.read_bytes()).hexdigest()
  if actual not in p['prior'].get(rel,[]):raise ValueError('unexpected destination version: '+rel)
root.mkdir(parents=True,exist_ok=True)
lock=os.open(root/'.sender.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
work=base/'tests/capture_origin'/('deploy-'+str(time.time_ns()));work.mkdir(parents=True)
old={rel:safe(rel).read_bytes() if safe(rel).exists() else None for rel in p['files']}
for rel,data in old.items():
 if data is not None:
  dest=work/'backup'/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
def put(path,data):
 path.parent.mkdir(parents=True,exist_ok=True)
 fd,tmp=tempfile.mkstemp(prefix='write-',dir=work)
 try:
  with os.fdopen(fd,'wb') as f:
   os.fchmod(f.fileno(),0o755 if path.suffix=='.sh' else 0o644);f.write(data);f.flush();os.fsync(f.fileno())
  os.replace(tmp,path)
 finally:
  if os.path.exists(tmp):os.unlink(tmp)
written=[]
try:
 for rel,text in p['files'].items():
  path=safe(rel)
  if path.exists() and path.read_text()==text:continue
  put(path,text.encode());written.append(rel)
 for rel,text in p['files'].items():assert safe(rel).read_bytes()==text.encode(),rel
 print(json.dumps(dict(root=str(base),written=written,motion_started=False,backup=str(work))))
except BaseException:
 for rel in written:
  if old[rel] is None:safe(rel).unlink()
  else:put(safe(rel),old[rel])
 raise
finally:os.close(lock)
"""
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--apply',action='store_true');args=ap.parse_args()
 files={};prior={}
 for line in (CHASSIS/'config/deploy-sha256.txt').read_text().splitlines():
  expected,rel=line.split(None,1);path=CHASSIS/rel
  if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('source checksum: '+rel)
  full='src/teleop/chassis/'+rel;files[full]=path.read_text()
  backup=ROOT/'tests/capture_origin/backups'/full
  if backup.exists():prior[full]=[hashlib.sha256(backup.read_bytes()).hexdigest()]
 files['src/teleop/chassis/config/deploy-sha256.txt']=(CHASSIS/'config/deploy-sha256.txt').read_text()
 backup=ROOT/'tests/capture_origin/backups/src/teleop/chassis/config/deploy-sha256.txt'
 prior['src/teleop/chassis/config/deploy-sha256.txt']=[hashlib.sha256(backup.read_bytes()).hexdigest()]
 files['configs/teleop/chassis.json']=(ROOT/'configs/teleop/chassis.json').read_text()
 previous=ROOT/'tests/capture_origin/previous-deployed-sha.json'
 if previous.exists():
  for rel,h in json.loads(previous.read_text()).items():prior.setdefault(rel,[]).append(h)
 if not args.apply:
  print(json.dumps(dict(destination='/home/user/gento-skye_scripts',files=list(files),motion_started=False)));return 0
 return subprocess.run(['ssh','-T','-o','BatchMode=yes','user@6.6.7.100','python3 -B -c '+shlex.quote(REMOTE)],input=json.dumps(dict(files=files,prior=prior)),text=True,timeout=40).returncode
if __name__=='__main__':raise SystemExit(main())
