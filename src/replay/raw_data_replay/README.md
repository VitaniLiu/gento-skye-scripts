# Raw replay implementation status

Robot-local episode input: `/home/user/gento-skye_raw_data/<task>/episodeXXXXXXXX`.
Default `main.py` is read-only; `--execute` remains unavailable until execution
integration is complete. Do not interpret successful native service responses
as evidence of actual IK output.

Verified: actual episode00000000 is readable and shared receive-time offsets
are retained. An isolated domain198 native playback probe accepts the absolute
`domain_24` bag path and start/stop commands. No IK samples were observed in two
8-second probes (default input_topic and control/ik_request). This result is
inconclusive: topic selection, graph discovery and QoS must be inspected before
claiming incompatibility. Hardware domain24 was not used by these probes.

Pending: native IK publication verification and timestamp correspondence;
gripper ownership transfer and slow-channel runtime; chassis publishing and
feedback-confirmed zero stop; shared startup barrier and all-component abort;
isolated integration and fault tests. Native output must be remapped or gated
until all channels are ready. Never overlap native and independent gripper
outputs. Existing recording files must remain unchanged.

Probe source: tests/raw_data_replay/native_probe.py. Test ROS logs are confined
to tests/raw_data_replay/ros_logs on the robot.
