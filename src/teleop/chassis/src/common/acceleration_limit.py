"""Pure command acceleration limiting; no ROS, I/O or feedback filtering.

This bounds speed increases, not emergency or controller-requested braking.
The limits describe commands, not a guarantee about physical acceleration.
"""
import math
from controller import Command


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class AccelerationLimiter:
    def __init__(self, linear_accel=.2, angular_accel=.4, started=None):
        if not all(_finite(v) and v > 0 for v in (linear_accel, angular_accel)):
            raise ValueError("acceleration limits must be positive and finite")
        self.linear_accel = linear_accel
        self.angular_accel = angular_accel
        self.reset(started)

    def reset(self, now=None):
        self.previous = Command()
        self.last_at = now if _finite(now) and now >= 0 else None

    @staticmethod
    def _axis(previous, target, limit, dt):
        # Braking is authoritative: never retain a forbidden turn or drive,
        # delay a lower approach speed, or accelerate through a reversal.
        if target == 0 or previous * target < 0:
            return 0.
        if abs(target) <= abs(previous):
            return target
        return math.copysign(min(abs(target), abs(previous) + limit * dt), target)

    def update(self, command, now):
        if (not isinstance(command, Command) or
                not all(_finite(v) for v in (command.vx, command.vy, command.wz)) or
                command.vy != 0):
            self.reset(now)
            raise ValueError("invalid command for acceleration limiter")
        if not _finite(now) or now < 0:
            self.reset()
            return Command(reason=command.reason)
        dt = None if self.last_at is None else now - self.last_at
        if dt is None or not 0 <= dt <= .100001:
            self.reset(now)
            return Command(reason=command.reason)
        # Do not accumulate idle or delayed scheduler time into a speed jump.
        dt = min(dt, .02)
        self.last_at = now
        output = Command(
            self._axis(self.previous.vx, command.vx, self.linear_accel, dt),
            0.,
            self._axis(self.previous.wz, command.wz, self.angular_accel, dt),
            command.reason)
        self.previous = output
        return output
