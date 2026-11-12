#!/usr/bin/env python3
"""Subscription-only endpoint: no publishers, service clients, reset or execution."""
import base64,json,select,sys,time
from alignment import ObservationHistory


def emit(value):print(json.dumps(value,allow_nan=False),flush=True)


def main():
    config=json.loads(sys.stdin.readline())
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray
    from move.msg import State
    from foxglove_msgs.msg import CompressedVideo
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init(domain_id=config["robot"]["ros_domain_id"])
    node=rclpy.create_node("gento_inference_observe_only")
    history=ObservationHistory();topics=config["topics"]
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    def joints(msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        history.append("joints",stamp,dict(zip(msg.name,msg.position)))
    def grip(side,msg):
        if msg.data:history.append(side+"_gripper",node.get_clock().now().nanoseconds,float(msg.data[0]))
    def base(msg):history.append("base",node.get_clock().now().nanoseconds,(float(msg.speed.linear.x),float(msg.speed.angular.z)))
    def camera(msg):
        stamp=msg.timestamp.sec*10**9+msg.timestamp.nanosec
        emit(dict(type="camera",timestamp_ns=stamp,data=base64.b64encode(bytes(msg.data)).decode()))
    node.create_subscription(JointState,topics["joints"],joints,qos)
    node.create_subscription(Float32MultiArray,topics["left_gripper_feedback"],lambda m:grip("left",m),qos)
    node.create_subscription(Float32MultiArray,topics["right_gripper_feedback"],lambda m:grip("right",m),qos)
    node.create_subscription(State,topics["chassis_state"],base,qos)
    node.create_subscription(CompressedVideo,topics["head_camera"],camera,qos)
    emit(dict(type="ready",mode="subscription_only"))
    try:
        while True:
            rclpy.spin_once(node,timeout_sec=.01)
            if not select.select([sys.stdin],[],[],0)[0]:continue
            line=sys.stdin.readline()
            if not line:break
            command=json.loads(line)
            if command["command"]=="quit":break
            if command["command"]!="observe":raise ValueError("read-only endpoint accepts observe/quit only")
            emit(history.snapshot(command.get("timestamp_ns")))
    except Exception as exc:
        emit(dict(type="error",error=str(exc)));return 2
    finally:
        node.destroy_node();rclpy.shutdown()
    return 0

if __name__=="__main__":raise SystemExit(main())
