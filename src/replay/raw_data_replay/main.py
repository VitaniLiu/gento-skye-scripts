"""Unified read-only preparation; hardware execution is deliberately gated."""
import argparse,json
from timeline import load,summary
import body_replay,gripper_replay,chassis_replay

def main():
    p=argparse.ArgumentParser()
    p.add_argument('episode',help='Robot-local episode directory')
    p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    if a.execute:p.error('Hardware backend incomplete: native start synchronization and gripper ownership transfer must be implemented before execution')
    plan=load(a.episode)
    body_replay.prepare(plan);gripper_replay.prepare(plan);chassis_replay.prepare(plan)
    print(json.dumps(summary(plan),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
