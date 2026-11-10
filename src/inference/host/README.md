# SKYE client boundary

Client observations: base_link-relative native flange_L/flange_R poses and
eyes_link, XYZ metres + first-two-column rotation6d, current 78D physical state.
Task mask and actual state_mask are separate. Existing waist protocol origin
0.4756409823894501 m is preserved; the server must use the same origin, not
infer it from the word physical. Gripper feedback is calibrated to [0,1] once.

Client responses: unnormalized native flange poses in base_link, column6d;
server must undo training normalization, camera C0 and Q exactly once. Client
never applies Q, a tool offset, or C0. info must echo the configured data_contract.
This check happens before robot startup; reset and infer carry the contract.
The existing server needs a SKYE adapter; this change does not implement it.

Images: left-eye RGB 1280x720; toulaji crop [286,0,1006,720), gouwu crop
[164,255,1179,704), then PIL Lanczos 320x320 with no letterbox/augmentation.
PNG base64 images.head_color is a list of five frames at [-.8,-.6,-.4,-.2,0]s.
image_timestamps_ns, image_target_timestamps_ns and image_padding_mask are lists
of five. Padding repeats earliest available past frame. No future frames.
Robot observation timestamp is the measured JointState header stamp. Image/state
sampling is causal, not a claim of physical sensor synchronization. Other tasks
need explicit crop configuration before using this SKYE input path.

Run tests/inference/test_skye_handoff.py for sample crop, history, data contract
and geometry tests. test/client_test/client_test.py convert emits skye_v3 protocol.
No startup motion, watchdog, rate limits or collision checks were added here.
Existing --execute, task mask, IK bounds and exit/reset behavior remain.
