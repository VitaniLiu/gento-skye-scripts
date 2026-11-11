"""Inference-owned synchronized joint trajectory, offline and live share this planner.

C1 monotone cubic interpolation, zero endpoint velocity, analytic velocity and
acceleration bounds, uniform time dilation. No robot/network imports or IO.
Acceleration settings are planning policy, not certified hardware limits.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.interpolate import PchipInterpolator, CubicHermiteSpline
from kinematics import JOINTS

@dataclass
class Plan:
    times: np.ndarray
    q: np.ndarray
    rows: list
    report: dict
    knots: np.ndarray
    knot_q: np.ndarray


def joint_vector(row):
    return np.asarray(sum(row["groups"], []) + row["body"], dtype=float)


def segment_extrema(spline):
    # Cubic y=a*u^3+b*u^2+c*u+d; velocity extrema at endpoints or -b/(3a).
    a,b,c,_ = spline.c
    h = np.diff(spline.x)[:,None]
    v0=c; v1=3*a*h*h+2*b*h+c
    u=np.divide(-b,3*a,out=np.zeros_like(b),where=np.abs(a)>1e-15)
    u=np.minimum(h,np.maximum(0,u))
    vm=3*a*u*u+2*b*u+c
    vmax=np.maximum.reduce([np.abs(v0),np.abs(v1),np.abs(vm)])
    amax=np.maximum(np.abs(2*b),np.abs(6*a*h+2*b))
    return vmax,amax


def extrema(spline):
    v,a=segment_extrema(spline)
    return v.max(axis=0),a.max(axis=0)


def curve(times,q):
    slopes=PchipInterpolator(times,q,axis=0).derivative()(times)
    slopes[[0,-1]]=0
    return CubicHermiteSpline(times,q,slopes,axis=0)


def plan_rows(rows, observation, kinematics, settings, calibration, startup_transition=True):
    spec=settings["trajectory"]
    dt=float(settings["action_dt_s"]);servo=float(spec["servo_dt_s"])
    seed=dict(zip(observation["joint_names"],observation["joint_positions"],strict=True))
    initial=np.asarray([seed[n] for n in JOINTS])
    targets=np.asarray([joint_vector(row) for row in rows])
    if not len(targets) or not np.isfinite(targets).all():raise ValueError("empty/nonfinite joint trajectory")
    limits=np.asarray([kinematics.limits[n] for n in JOINTS])
    low,high,speed=limits.T
    if np.any(targets<low-1e-7) or np.any(targets>high+1e-7):raise ValueError("joint target outside URDF position limits")
    if np.any(initial<low-1e-6) or np.any(initial>high+1e-6):raise ValueError("measured start outside URDF limits")
    speed=speed*float(spec["joint_velocity_scale"])
    speed[16]=min(speed[16],float(spec["waist_height_velocity_m_s"]))
    accel=speed*float(spec["joint_acceleration_per_velocity_s"])
    delta=np.abs(targets[0]-initial)
    entry=max(float(spec["startup_min_s"]) if startup_transition else servo,
              float(np.max(1.875*delta/speed)),
              float(np.max(np.sqrt(5.78*delta/accel))))
    # Entry -> first target; then retain all N model intervals (first is a hold).
    knot_q=np.vstack([initial,targets[0],targets])
    steps=np.diff(np.vstack([targets[0],targets]),axis=0)
    model_intervals=np.maximum(dt,np.max(1.5*np.abs(steps)/speed,axis=1))
    intervals=np.r_[entry,model_intervals]
    knots=np.r_[0,np.cumsum(intervals)]
    original_intervals=intervals.copy()
    # Dilate only offending intervals, not the entire chunk for one short corner.
    for iteration in range(60):
        spline=curve(knots,knot_q)
        segment_v,segment_a=segment_extrema(spline)
        ratios=np.maximum(np.max(segment_v/speed,axis=1),
                          np.sqrt(np.max(segment_a/accel,axis=1)))
        if ratios.max() <= 1.000001:
            break
        intervals*=np.where(ratios>1.,ratios*1.01,1.)
        knots=np.r_[0,np.cumsum(intervals)]
    else:
        raise ValueError("trajectory retiming did not converge")
    vmax,amax=extrema(spline)
    dilation=float(np.max(intervals/original_intervals))
    times=np.r_[np.arange(0,knots[-1],servo),knots[-1]]
    q=spline(times)
    if np.any(q<low-1e-6) or np.any(q>high+1e-6):raise ValueError("interpolation exceeded joint limits")
    # Base displacement and yaw are integrated at original model dt, then follow
    # the SAME retimed knots. Prefix displacement is zero. The derivative gives
    # retimed velocity, preserving each full chunk's integrated displacement.
    vx=np.array([row["vx"] for row in rows]);wz=np.array([row["wz"] for row in rows])
    distance=np.vstack([np.zeros(2),np.zeros(2),np.cumsum(np.c_[vx,wz]*dt,axis=0)])
    distance_samples=curve(knots,distance)(times)
    base=np.zeros_like(distance_samples)
    base[:-1]=np.diff(distance_samples,axis=0)/np.diff(times)[:,None]
    # Gripper interpolation shares the time map; masked grippers stay None.
    grippers=[]
    for key,side in [("left_gripper","left"),("right_gripper","right")]:
        if all(row[key] is None for row in rows):grippers.append(None);continue
        bounds=calibration[side]
        first=float(np.clip((observation[key]-bounds["closed"])/(bounds["open"]-bounds["closed"]),0,1))
        values=[first]
        for row in rows:values.append(values[-1] if row[key] is None else row[key])
        grip=np.r_[values[0],values[1],values[1:]]
        grippers.append(PchipInterpolator(knots,grip)(times))
    output=[]
    for i,(stamp,value) in enumerate(zip(times,q)):
        output.append(dict(time_s=float(stamp),groups=[value[:7].tolist(),value[7:14].tolist(),value[14:16].tolist()],
            body=value[16:19].tolist(),vx=float(base[i,0]),wz=float(base[i,1]),
            left_gripper=None if grippers[0] is None else float(grippers[0][i]),
            right_gripper=None if grippers[1] is None else float(grippers[1][i])))
    raw_delta=np.diff(np.vstack([initial,targets]),axis=0)
    report=dict(source_frames=len(rows),servo_samples=len(times),servo_hz=1/servo,
        source_duration_s=len(rows)*dt,startup_transition=bool(startup_transition),
        startup_duration_s=float(knots[1]),
        rollout_duration_s=float(knots[-1]-knots[1]),total_duration_s=float(knots[-1]),
        dilation=float(dilation),raw_max_urdf_velocity_ratio=float(np.max(np.abs(raw_delta)/dt/limits[:,2])),
        planned_max_urdf_velocity_ratio=float(np.max(vmax/limits[:,2])),
        max_policy_velocity_ratio=float(np.max(vmax/speed)),max_policy_acceleration_ratio=float(np.max(amax/accel)),
        joint_names=list(JOINTS),velocity_limits=speed.tolist(),acceleration_policy=accel.tolist(),
        interpolation="C1 monotone cubic; zero endpoint velocity; no Cartesian path guarantee",
        base_displacement_m=float(distance[-1,0]),base_yaw_rad=float(distance[-1,1]))
    return Plan(times,q,output,report,knots,knot_q)
