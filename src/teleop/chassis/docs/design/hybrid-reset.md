# Hybrid reverse return — approved 2026-11-02
The user chose reverse command order plus measured trajectory feedback.
Manual motion is grouped into ordered drive/turn legs. Each leg includes measured
coasting after zero commands. A reverses leg order and drive direction. Position
and yaw feedback decide completion, rather than recorded command duration.
Straight/turn speed caps are 0.05 m/s and 0.1 rad/s. Existing operator cancellation,
input/feedback freshness, path corridor and time/no-progress stops remain active.
Initial rotations remain in sequence. Straight legs converge heading while moving,
then command zero yaw for their final 4 cm. No terminal wheel-straightening pulse or
standalone terminal heading rotation is added.
Alternatives: pure time reversal was rejected after the 43.92 mm real result.
Legacy position following achieved 5.15 mm but lacked command-order constraints.
This implementation combines their intent; it does not promise zero physical error.
Validation: full tests, the 12:21 success recording plus recent recordings with
different speed gains, and quantized odometry/lag stress tests. Only then sync
scoped files and run sender --check with publisher disabled. The user performs
subsequent physical testing.
