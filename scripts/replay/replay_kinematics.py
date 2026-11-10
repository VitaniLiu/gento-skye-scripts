"""Replay-local read-only URDF FK with fixed MJCF tool extensions."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def error(reason):
    raise ValueError(f"{Path(__file__).resolve()}:Kinematics: {reason}")

class Kinematics:
    def __init__(self, urdf, mjcf, hashes=None):
        self.paths=[Path(urdf),Path(mjcf)]
        for p in self.paths:
            if hashes and hashlib.sha256(p.read_bytes()).hexdigest()!=hashes.get(p.name):
                error(f"model hash mismatch: {p}")
        tree=ET.parse(urdf).getroot()
        self.joints=[]
        for j in tree.findall("joint"):
            o=j.find("origin");axis=j.find("axis")
            xyz=np.fromstring(o.get("xyz","0 0 0") if o is not None else "0 0 0",sep=" ")
            rpy=np.fromstring(o.get("rpy","0 0 0") if o is not None else "0 0 0",sep=" ")
            m=np.eye(4);m[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix();m[:3,3]=xyz
            self.joints.append((j.get("name"),j.get("type"),j.find("parent").get("link"),j.find("child").get("link"),m,np.fromstring(axis.get("xyz") if axis is not None else "1 0 0",sep=" ")))
        self.required={j[0] for j in self.joints if j[1]!="fixed"}
        self.extensions=[]
        root=ET.parse(mjcf).getroot();parents={c:p for p in root.iter() for c in p}
        for name in ("left_tool","right_tool","eyes_link"):
            body=next(b for b in root.iter("body") if b.get("name")==name)
            if body.find("joint") is not None:error("tool extension is not fixed")
            w,x,y,z=np.fromstring(body.get("quat","1 0 0 0"),sep=" ")
            m=np.eye(4);m[:3,:3]=Rotation.from_quat([x,y,z,w]).as_matrix()
            m[:3,3]=np.fromstring(body.get("pos","0 0 0"),sep=" ")
            self.extensions.append((parents[body].get("name"),name,m))

    def forward(self, names, positions):
        if len(names)!=len(positions) or len(set(names))!=len(names):error("joint names/positions mismatch or duplicate names")
        q=dict(zip(names,positions))
        missing=self.required-set(q)
        if missing:error(f"missing joints: {sorted(missing)}")
        if not np.isfinite(list(q.values())).all():error("nonfinite joint value")
        frames={"base_link":np.eye(4)};pending=self.joints.copy()
        while pending:
            done=[]
            for j in pending:
                name,kind,parent,child,origin,axis=j
                if parent not in frames:continue
                motion=np.eye(4)
                if kind in ("revolute","continuous"):
                    motion[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*q[name]).as_matrix()
                elif kind=="prismatic":motion[:3,3]=axis*q[name]
                elif kind!="fixed":error(f"unsupported joint type: {kind}")
                frames[child]=frames[parent]@origin@motion;done.append(j)
            if not done:error("disconnected/cyclic kinematic tree")
            pending=[j for j in pending if all(j is not x for x in done)]
        for parent,name,m in self.extensions:frames[name]=frames[parent]@m
        return frames

    def poses78(self, names, positions):
        frames=self.forward(names,positions)
        return {name:np.concatenate((frames[name][:3,3],frames[name][:2,:3].reshape(6))).astype(np.float32)
                for name in ("left_tool","right_tool","eyes_link")}
