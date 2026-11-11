# SKYE client boundary

Client observations: base_link-relative native flange_L/flange_R poses and
eyes_link, XYZ metres + first-two-column rotation6d, current 78D physical state.
Task mask and actual state_mask are separate. Existing waist protocol origin
0.4756409823894501 m is preserved; the server must use the same origin, not
infer it from the word physical. Gripper feedback is calibrated to [0,1] once.

Client responses: unnormalized native flange poses in base_link, column6d;
server must undo training normalization, camera C0 and Q exactly once. Client
never applies Q, a tool offset, or C0. The server is not required to return data_contract.
Reset and infer carry descriptive metadata; its semantics are agreed out of band.
The existing server needs a SKYE adapter; this change does not implement it.

Images: left-eye RGB 1280x720; toulaji crop [286,0,1006,720), gouwu crop
[164,255,1179,704), then PIL Lanczos 320x320 with no letterbox/augmentation.
The default legacy input uses five PNG base64 images.head_color frames at
[-.8,-.6,-.4,-.2,0]s; model deployments may select current-only [0].
image_timestamps_ns, image_target_timestamps_ns and image_padding_mask match
the selected frame count. Padding repeats earliest available past frame. No future frames.
The host first obtains numeric coverage, selects the latest decoded image at or
before that cutoff as t0, then requests numeric snapshots at that exact t0.
Robot buffers select the most recent sample <= t0 for joints, grippers and base.
Joint samples use ROS header time; unstamped gripper/base use robot ROS receive
time. Source timestamps/bases travel with obs. Five history images share t0.
This is causal software alignment, not physical sensor synchronization. Other tasks
need explicit crop configuration before using this SKYE input path.

Run tests/inference/test_skye_handoff.py for sample crop, history, data contract
and geometry tests. test/client_test/client_test.py convert emits skye_v3 protocol.
No startup motion, watchdog, rate limits or collision checks were added here.
Existing --execute, task mask, IK bounds and exit/reset behavior remain.

Waist reference is verified against the supplied sample and original episode10:
raw feedback 0.3145900070667267 - sample(-0.1610509753227234) =
0.4756409823894501 m protocol origin. Startup reset remains the capture default
0.49631398916244507 m; it is not a replacement protocol origin.

Model selection: `--model NAME` loads `configs/inference/deployed_NAME.json`.
That deployment selects SSH or direct TCP transport, the server endpoint and a
task registry through `tasks_file`; the deployment's `model` value is sent in
reset and infer, so NAME may be a checkpoint/profile alias.
`instruction_mode=request` sends the selected task's instruction in every reset
and infer request. `instruction_mode=server` sends `server_task` instead and
requires the prompt reported by server info to match the local task registry.


Eight-model deployment (2026-11-11):
`--model pi05|alpha|fastwam|groot|ours_direct|ours_pre|ours_a0|ours_a1`
selects ports 19300..19307 respectively at 10.0.0.203 via zw.
These endpoints are recorded deployment values, not a live availability claim.
All use tasks_eightpost.json with the supplied English training instructions.
Task identifiers remain gouwu and toulaji without an embodiment prefix.
Server instruction mode verifies the configured prompt via info; infer sends task.
The four baselines select offsets [0]; four Ours select [-.8,-.6,-.4,-.2,0].
Only current state is sent. Native v4 transport uses cmd, float32 state arrays,
uint8 head_rgb and, for Ours, head_rgb_history, history_offsets and history_is_pad.
Existing crop/Lanczos/PNG preprocessing is retained; native transport decodes PNG
back to lossless RGB arrays before array serialization. No PNG gateway is required.
state_frame=source and embodiment=skye; no client Q or camera transform is added.
Server must accept these task identifiers and native flange axes; no server changes
or live inference validation are included in this client implementation.
Nested server error responses stop the request, including the initial info call.
Example configuration check (no network or motion):
`scripts/inference/inference --model ours_a1 --task toulaji --steps 32`
Add --execute only for an explicitly intended robot run after server integration.


## 2026-11-11 once-per-launch task startup alignment

After the existing runtime Home/waist reset, impedance mode and parking release,
SKYE tasks with `startup` in tasks_eightpost.json align once per client launch to
the task DATA observation `state` at frame 0 (not an action target). The configured
native base_link flange/head state is unnormalized column6d, with the existing
relative waist origin. Source episode/frame and float32 state SHA-256 are recorded
in the task configuration. Gouwu uses original episode00000000; toulaji uses
original episode00000010. No image or dataset file is required on deployment.

The host compares enabled pose, waist and calibrated gripper coordinates. Base
vx/wz are always zero during this transition, regardless of DATA velocities;
masked coordinates and unused finger fields are ignored. Existing response IK
solves the target; the existing C1 planner joins measured joints to it at 100 Hz,
with zero endpoint velocity, at least 3 seconds entry, 0.5 x URDF joint velocity,
waist <=0.03 m/s and the existing acceleration policy. Runtime uniform 10 ms
scheduling may stretch the last planner interval by less than 10 ms. The solved
target is checked before execution; measured arrival is required afterwards.

Declared arrival tolerances: arms 3 mm, head 5 mm, pose rotation 0.03 rad,
active waist height 1 mm/angles 0.01 rad, and grippers 0.03 fraction. Arrival
polling times out after 5 seconds; failure or q/cancel prevents policy reset and
inference and follows the existing body/waist/chassis cleanup. This is a joint
space transition with no Cartesian path or collision guarantee.

After arrival (including an already-aligned pose), buffered images are cleared
and late camera packets at/before the arrival timestamp are rejected. A fresh
causally aligned observation/image sample is acquired before policy reset and
used for the first infer; leading history slots pad from the earliest fresh
frame. Startup alignment never repeats between action chunks. Policy chunks
retain direct 20 Hz execution; tasks without a startup specification retain
legacy startup behavior. Development checks are offline only; no hardware motion
or control endpoint is started by tests/inference/test_startup_alignment.py.
