"""Synchronize only this project's layered runtime; never start motion."""
from pathlib import Path
import hashlib,json,subprocess,shlex,sys
ROOT=Path(__file__).resolve().parents[1]
REMOTE=r"""
from pathlib import Path
import json,hashlib,os,tempfile,tarfile,time,sys
p=json.load(sys.stdin);root=Path('/home/user/tianjidipan').resolve()
sha=lambda b:hashlib.sha256(b).hexdigest()
files=p['files'];legacy=p['legacy']
def safe(rel):
 path=root/rel
 if Path(rel).is_absolute() or '..' in Path(rel).parts or path.is_symlink() or not path.resolve().is_relative_to(root):
  raise SystemExit('Unsafe target: '+rel)
 return path
already=all(safe(n).is_file() and sha(safe(n).read_bytes())==sha(t.encode()) for n,t in files.items())
for rel,expected in legacy.items():
 path=safe(rel)
 if path.exists():
  if not path.is_file() or sha(path.read_bytes())!=expected:
   if not (rel in files and sha(path.read_bytes())==sha(files[rel].encode())):
    raise SystemExit('Unexpected legacy file: '+rel)
 elif not already:raise SystemExit('Missing expected legacy file: '+rel)
for rel,text in files.items():
 path=safe(rel)
 if path.exists() and sha(path.read_bytes())!=sha(text.encode()) and rel not in legacy:
  raise SystemExit('Unexpected existing destination: '+rel)
paths=set(files)|set(legacy)
saved={n:(safe(n).read_bytes(),safe(n).stat().st_mode&0o777) if safe(n).exists() else None for n in paths}
written=[];removed=[]
if not already:
 backup=root/'backups'/('before-src-layout-'+time.strftime('%Y%m%d-%H%M%S')+'.tar.gz')
 backup.parent.mkdir(exist_ok=True)
 with tarfile.open(backup,'w:gz') as tar:
  for n in legacy:
   if safe(n).exists():tar.add(safe(n),arcname=n,recursive=False)
try:
 for rel,text in files.items():
  path=safe(rel);data=text.encode()
  if path.exists() and path.read_bytes()==data:continue
  path.parent.mkdir(parents=True,exist_ok=True)
  fd,tmp=tempfile.mkstemp(prefix='.layout-',dir=path.parent)
  try:
   os.fchmod(fd,0o755 if rel.endswith('.sh') else 0o644)
   with os.fdopen(fd,'wb') as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
   os.replace(tmp,path);written.append(rel)
  finally:
   if os.path.exists(tmp):os.unlink(tmp)
 for rel,text in files.items():assert safe(rel).read_bytes()==text.encode(),rel
 for rel in legacy:
  if rel not in files and safe(rel).exists():
   safe(rel).unlink();removed.append(rel)
 for name in ['reset']:
  path=root/name
  if path.is_dir() and not any(path.iterdir()):path.rmdir()
 print(json.dumps(dict(type='layout_deployed',written=written,legacy_removed=removed,motion_started=False)))
except BaseException:
 for rel in set(written+removed):
  path=safe(rel);old=saved[rel]
  if old is None:
   if path.exists():path.unlink()
  else:
   path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(old[0]);path.chmod(old[1])
 raise
"""
def main():
    import argparse
    parser=argparse.ArgumentParser(description="Verify local deployment payload; --apply explicitly checks and updates robot files without starting motion")
    parser.add_argument("--apply",action="store_true")
    args=parser.parse_args()
    baseline=json.loads((ROOT/'config/layout-baseline.json').read_text())['legacy_robot_sha256']
    lines=(ROOT/'config/deploy-sha256.txt').read_text().splitlines()
    files={}
    for line in lines:
        expected,rel=line.split(None,1)
        data=(ROOT/rel).read_bytes()
        if hashlib.sha256(data).hexdigest()!=expected:
            raise SystemExit('Local checksum differs: '+rel)
        files[rel]=data.decode()
    files['config/deploy-sha256.txt']=(ROOT/'config/deploy-sha256.txt').read_text()
    if not args.apply:
        print(json.dumps(dict(source=str(ROOT),destination='/home/user/tianjidipan',files={r:hashlib.sha256(t.encode()).hexdigest() for r,t in files.items()},remote_compared=False,motion_started=False,note='Use --apply only for an intentional robot update; remote baseline/differences are checked before writes')))
        return 0
    command='flock -n /home/user/tianjidipan/.sender.lock python3 -B -c '+shlex.quote(REMOTE)
    result=subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8','user@6.6.7.100',command],
                          input=json.dumps(dict(legacy=baseline,files=files)),text=True,timeout=60)
    if result.returncode:return result.returncode
    return subprocess.run(['bash',str(ROOT/'check.sh')]).returncode
if __name__=='__main__':raise SystemExit(main())
