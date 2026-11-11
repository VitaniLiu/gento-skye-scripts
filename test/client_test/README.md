# Recorded Episode protocol test

This test does not connect to an inference server. It converts robot-local raw
MCAP into the inference client's request/response JSON objects, then optionally
feeds the recorded response objects through the production mask, IK, and robot
dispatch path.

Convert a complete Episode. This is read-only and does not move the robot:

```bash
.venv/bin/python test/client_test/client_test.py \
  --task toulaji convert --episode 4
```

The default output is:

```text
test/client_test/data/toulaji_episode00000004.skye_v3.protocol.jsonl
```

Replay an existing protocol file. This resets the robot, switches to impedance
mode, releases chassis parking, replays every recorded action and mask at 20 Hz,
then resets the upper body/waist and returns the chassis to its start:

```bash
.venv/bin/python test/client_test/client_test.py \
  --task toulaji --execute replay \
  --protocol-file test/client_test/data/toulaji_episode00000004.skye_v3.protocol.jsonl
```

Convert and immediately replay:

```bash
.venv/bin/python test/client_test/client_test.py \
  --task toulaji --execute run --episode 4
```

During `replay` or `run`, press `q` to stop. Cleanup still performs the normal
body/waist reset followed by chassis return. `--protocol-file PATH` may be used
with `convert` or `run` to override the default output path.

The JSONL header records the task, source Episode, masks, timing, dimensions and
conversion alignment. Each following line contains one exact client request and
one server-shaped response whose actions and masks came from the recorded raw
control targets. No server-generated `response.json` is used.

## SKYE native-wrist protocol v3

Observation uses recorded measured joints and production native-flange FK.
Action uses recorded controller command joints and the same FK, matching the
handoff training source; target_poseL/R is no longer substituted for this source.
No Q or tool offset is applied. Head remains eyes_link.

Both convert and production inference share task crop + Lanczos resize to
320x320, PNG RGB, and five causal history images at [-.8,-.6,-.4,-.2,0] seconds.
The request contains actual/target timestamps, padding flags, and state_mask.
State is current only. Missing historical images repeat the earliest available
past image. The test protocol records data_contract and image_spec.

Old v1/v2 protocols are rejected. No source data, shared processing code or
LeRobot dataset is modified. convert does not contact a policy server or move
the robot. replay/run retain --execute and normal reset/quit behavior.

The handoff ZIP base-wrist arrays are Q-aligned, despite being in base coordinates.
They must have Q undone once by the server adapter before native-flange IK.
Do not feed sample arrays.action (camera frame/row6d) to this client directly.
