"""Run the native Gento player on one robot-local domain_24 bag."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    bag = args.episode.resolve() / "domain_24"
    if not (bag / "metadata.yaml").is_file():
        parser.error(f"找不到录制：{bag}")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from marvin_msgs.srv import GentoReplay, Int
    from std_msgs.msg import String

    rclpy.init(domain_id=24, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("raw_native_launcher")
    tag = "raw_direct_" + uuid.uuid4().hex[:8]
    status = {}
    def on_status(msg):
        status.update(json.loads(msg.data))
    node.create_subscription(String, f"/{tag}/status", on_status, 10)
    player = node.create_client(GentoReplay, f"/{tag}/playback")
    mux = node.create_client(Int, "/tj/control/set_input")

    def call(client, request):
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError(f"服务不可用：{client.srv_name}")
        result = client.call_async(request)
        rclpy.spin_until_future_complete(node, result, timeout_sec=8)
        if not result.done():
            raise RuntimeError(f"服务响应超时：{client.srv_name}")
        reply = result.result()
        if not reply.success:
            raise RuntimeError(reply.message)
        return reply

    command = ["/opt/kernelmind/apex/install/gento_replay/lib/gento_replay/gento_replay_node",
               "--ros-args", "-r", "__ns:=/tj", "-r", f"__node:={tag}",
               "-p", "ros_namespace:=tj", "-p", "bag_topic:=/tj/control/ik_request",
               "-p", f"input_topic:=/{tag}/unused_input", "-p", "loop_playback:=false",
               "-p", "record_camera:=false"]
    for name, target in (("control/gento_replay/playback_named", "playback"),
                         ("control/gento_replay/playback", "playback_int"),
                         ("info/gento_replay/status", "status")):
        command += ["-r", f"{name}:=/{tag}/{target}"]
    proc = None
    started = False
    try:
        proc = subprocess.Popen(command, start_new_session=True)
        if not player.wait_for_service(timeout_sec=8):
            raise RuntimeError("原生播放器未就绪")
        call(mux, Int.Request(data=1))
        started = True
        reply = call(player, GentoReplay.Request(data=1, name=str(bag)))
        print(reply.message, flush=True)
        deadline = time.monotonic() + 8
        active = False
        while True:
            if proc.poll() is not None:
                raise RuntimeError("原生播放器退出")
            rclpy.spin_once(node, timeout_sec=.1)
            if status.get("playback_active"):
                active = True
            elif active:
                print("原生回放结束", flush=True)
                return
            elif time.monotonic() >= deadline:
                raise RuntimeError("原生播放器未进入播放状态")
    except KeyboardInterrupt:
        print("停止原生回放", flush=True)
    finally:
        if started:
            try:
                call(player, GentoReplay.Request(data=0, name=str(bag)))
            except (OSError, RuntimeError) as exc:
                print(f"停止服务：{exc}", flush=True)
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.default_int_handler)
    try:
        main()
    except (RuntimeError, OSError) as exc:
        raise SystemExit(str(exc))
