"""Self-contained robot-side endpoint; executed over SSH from this package only."""
import json
from pathlib import Path
import select
import sys
import time
import xml.etree.ElementTree as ET


ROBOT_URDF = Path("/opt/kernelmind/apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky.urdf")
JOINT_GROUPS = (
    tuple(f"Joint{i}_L" for i in range(1, 8)),
    tuple(f"Joint{i}_R" for i in range(1, 8)),
    ("Head_J1_Joint", "Head_J2_Joint"),
)
WAIST = tuple(f"waist_joint_{i}" for i in range(1, 4))
JOINTS = sum(JOINT_GROUPS, ()) + WAIST


def smoothstep5(value):
    """Quintic blend with zero velocity and acceleration at both ends."""
    value = min(1.0, max(0.0, float(value)))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def move_duration(current, target, names, limits, speed_fraction, minimum_s):
    """Duration that keeps quintic peak speed below a URDF velocity fraction."""
    if not 0.0 < speed_fraction <= 0.25:
        raise ValueError("move-to-start speed fraction must be in (0, 0.25]")
    duration = float(minimum_s)
    for before, after, name in zip(current, target, names, strict=True):
        velocity = limits[name][2] * speed_fraction
        if velocity <= 0.0:
            raise ValueError(f"nonpositive installed velocity limit: {name}")
        # The maximum derivative of quintic smoothstep is 1.875.
        duration = max(duration, 1.875 * abs(after - before) / velocity)
    return duration


def installed_limits():
    root = ET.parse(ROBOT_URDF).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    return {
        name: tuple(float(joints[name].find("limit").get(key)) for key in ("lower", "upper", "velocity"))
        for name in JOINTS if name in joints and joints[name].find("limit") is not None
    }


def robot_main():
    import rclpy
    from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
    from marvin_msgs.srv import Int
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32
    from std_srvs.srv import SetBool

    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"], domain_id=24,
               signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("cross_embodiment_replay")
    joints = {}
    feedback = {}
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(JointState, "/tj/joint_states",
                             lambda message: joints.update(zip(message.name, message.position)), sensor_qos)
    node.create_subscription(Jointfeedback, "/tj/info/joint_feedback",
                             lambda message: feedback.update(
                                 body=list(message.body_positions), head=message.head_positions[2]), sensor_qos)
    command_mux = node.create_client(Int, "/tj/control/set_input")
    grip_mux = node.create_client(SetBool, "/tj/control/gripper_replay/enable")

    def switch(client, request):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"service failed: {client.srv_name}")

    deadline = time.monotonic() + 4
    while not all(name in joints for name in JOINTS) or not feedback:
        if time.monotonic() >= deadline:
            raise RuntimeError("complete robot joint feedback unavailable")
        rclpy.spin_once(node, timeout_sec=.1)
    print(json.dumps({name: joints[name] for name in JOINTS}), flush=True)
    line = sys.stdin.readline()
    if not line:
        node.destroy_node()
        return
    payload = json.loads(line)
    rows = payload["rows"]
    expected_limits = {name: tuple(values) for name, values in payload["limits"].items()}
    actual_limits = installed_limits()
    for name, expected in expected_limits.items():
        actual = actual_limits.get(name)
        if actual is None or max(abs(a - b) for a, b in zip(actual, expected, strict=True)) > 1e-7:
            raise RuntimeError(f"installed Gento Skye limit mismatch: {name}: {actual} != {expected}")
    for _, groups, commands in rows:
        if any(abs(value) > 1e-8 for value in commands[:4]) or commands[4] is not None:
            raise RuntimeError("cross-embodiment replay requires fixed zero base/grippers and fixed waist")
        for group, names in zip(groups, JOINT_GROUPS, strict=True):
            for value, name in zip(group, names, strict=True):
                low, high, _ = expected_limits[name]
                if not low - 1e-8 <= value <= high + 1e-8:
                    raise RuntimeError(f"trajectory exceeds installed joint limit: {name}={value}")
    tolerance = float(payload["start_tolerance_rad"])
    speed_fraction = float(payload["move_to_start_speed_fraction"])
    control_hz = float(payload["move_to_start_control_hz"])
    minimum_s = float(payload["move_to_start_min_duration_s"])
    maximum_s = float(payload["move_to_start_max_duration_s"])
    settle_s = float(payload["move_to_start_settle_s"])
    if not 20.0 <= control_hz <= 100.0:
        raise ValueError("move-to-start control frequency must be in [20, 100] Hz")
    if minimum_s < 0.0 or maximum_s <= 0.0 or not 0.0 <= settle_s <= 5.0:
        raise ValueError("invalid move-to-start timing configuration")
    first = sum(rows[0][1], [])
    replay_names = sum(JOINT_GROUPS, ())
    current = [joints[name] for name in sum(JOINT_GROUPS, ())]
    transition_s = move_duration(
        current, first, replay_names, expected_limits, speed_fraction, minimum_s)
    if transition_s > maximum_s:
        raise RuntimeError(
            f"move-to-start requires {transition_s:.3f}s, above configured {maximum_s:.3f}s")

    types = (JointcmdArm, JointcmdArm, JointcmdHead, JointcmdBody)
    publishers = [node.create_publisher(kind, "/tj/control/user/joint_cmd_" + suffix, 1)
                  for kind, suffix in zip(types, ("A", "B", "head", "body"), strict=True)]
    grip = [node.create_publisher(Float32, "/tj/control/gripper_replay_target" + side, 1) for side in "LR"]
    heartbeat = node.create_publisher(Bool, "/tj/control/gripper_replay/heartbeat", 1)
    body_hold = [joints[name] for name in WAIST] + feedback["body"][3:]
    head_hold = feedback["head"]
    armed = grippers = False

    def publish(groups):
        stamp = node.get_clock().now().to_msg()
        positions = (groups[0], groups[1], groups[2] + [head_hold], body_hold)
        for publisher, kind, values in zip(publishers, types, positions, strict=True):
            message = kind(positions=values)
            message.header.stamp = stamp
            publisher.publish(message)

    def publish_grippers_zero():
        heartbeat.publish(Bool(data=True))
        grip[0].publish(Float32(data=0.0))
        grip[1].publish(Float32(data=0.0))

    def grouped(flat):
        return [flat[:7], flat[7:14], flat[14:16]]

    def wait_until(deadline):
        remaining = max(0.0, deadline - time.monotonic())
        if select.select([sys.stdin], [], [], remaining)[0]:
            return False
        rclpy.spin_once(node, timeout_sec=0.0)
        return True

    try:
        # Fill the command topics with the measured pose before selecting this
        # source, avoiding a stale command when the mux changes ownership.
        for _ in range(5):
            publish(grouped(current))
            if not wait_until(time.monotonic() + 0.02):
                return
        switch(command_mux, Int.Request(data=3))
        armed = True
        switch(grip_mux, SetBool.Request(data=True))
        grippers = True

        # Move from measured joints to the clipped first frame using a quintic
        # joint-space path. Duration is derived from installed URDF velocities.
        period = 1.0 / control_hz
        begin = time.monotonic()
        step = 0
        while True:
            elapsed = min(transition_s, time.monotonic() - begin)
            blend = smoothstep5(1.0 if transition_s == 0.0 else elapsed / transition_s)
            command = [a + blend * (b - a) for a, b in zip(current, first, strict=True)]
            publish(grouped(command))
            publish_grippers_zero()
            if elapsed >= transition_s:
                break
            step += 1
            if not wait_until(begin + min(transition_s, step * period)):
                return

        settle_end = time.monotonic() + settle_s
        while time.monotonic() < settle_end:
            publish(rows[0][1])
            publish_grippers_zero()
            if not wait_until(min(settle_end, time.monotonic() + period)):
                return
        errors = {name: abs(joints[name] - target) for name, target in zip(replay_names, first, strict=True)}
        worst_name = max(errors, key=errors.get)
        if errors[worst_name] > tolerance:
            raise RuntimeError(
                f"move-to-start did not converge: {worst_name} error "
                f"{errors[worst_name]:.6f} rad > {tolerance:.6f} rad")

        begin = time.monotonic()
        for timestamp, groups, _ in rows:
            if not wait_until(begin + timestamp):
                break
            if time.monotonic() - (begin + timestamp) > .1:
                raise RuntimeError("replay is more than 100 ms late")
            publish(groups)
            publish_grippers_zero()
    finally:
        if armed:
            switch(command_mux, Int.Request(data=1))
        if grippers:
            switch(grip_mux, SetBool.Request(data=False))
        node.destroy_node()


if __name__ == "__main__" and sys.argv[1:] == ["--robot"]:
    robot_main()
