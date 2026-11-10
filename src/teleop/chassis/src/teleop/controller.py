"""Pure offline D-pad mapping. No transport, ROS, service or actuator access."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class InputFrame:
    axes: tuple
    buttons: tuple
    connected: bool = True
    usb_age_s: float = 0.0
    network_age_s: float = 0.0

@dataclass(frozen=True)
class SafetyState:
    armed: bool = False
    software_manual: bool = False
    feedback_age_s: float = float("inf")

@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    reason: str = "disabled"

@dataclass(frozen=True)
class PressRecord:
    direction: str
    pressed_at: str
    released_at: str
    duration_s: float
    complete: bool
    end_reason: str

@dataclass(frozen=True)
class StepResult:
    command: Command
    events: tuple = ()

@dataclass
class _Hold:
    direction: str
    started: float
    timestamp: str
    start_observed: bool


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class DpadController:
    """Explicit simulated safety inputs; returned targets are never transmitted."""
    def __init__(self):
        self._released = False
        self._engaged = False
        self._holds = {}
        self._previous_fresh = False
        self._last_now = None

    def _reset_gate(self):
        self._released = self._engaged = False

    def _close(self, axis, now, timestamp, reason, complete):
        hold = self._holds.pop(axis)
        return PressRecord(hold.direction, hold.timestamp, timestamp,
                           round(max(0.0, now - hold.started), 9),
                           complete and hold.start_observed, reason)

    def _close_all(self, now, timestamp, reason):
        events = tuple(self._close(axis, now, timestamp, reason, False)
                       for axis in tuple(self._holds))
        self._previous_fresh = False
        return events

    @staticmethod
    def _valid_input(frame):
        try:
            return (isinstance(frame, InputFrame)
                    and isinstance(frame.axes, (tuple, list)) and isinstance(frame.buttons, (tuple, list))
                    and len(frame.axes) == 8 and len(frame.buttons) == 13
                    and all(_number(v) and -1 <= v <= 1 for v in frame.axes)
                    and all(type(v) in (int, bool) and v in (0, 1) for v in frame.buttons)
                    and type(frame.connected) is bool
                    and all(_number(v) and v >= 0
                            for v in (frame.usb_age_s, frame.network_age_s)))
        except (TypeError, AttributeError):
            return False

    def update(self, frame, safety, now, timestamp):
        if not _number(now) or now < 0 or (self._last_now is not None and now < self._last_now):
            end = self._last_now if self._last_now is not None else 0.0
            events = self._close_all(end, timestamp, "invalid_clock")
            self._reset_gate()
            return StepResult(Command(reason="invalid_clock"), events)
        self._last_now = now
        if not self._valid_input(frame):
            events = self._close_all(now, timestamp, "invalid_input")
            self._reset_gate()
            return StepResult(Command(reason="invalid_input"), events)
        fresh = (frame.connected and frame.usb_age_s < .15 and frame.network_age_s < .15)
        if not fresh:
            events = self._close_all(now, timestamp, "input_lost")
            self._reset_gate()
            return StepResult(Command(reason="disabled_or_stale"), events)

        # Track physical full-range holds independently from permission to move.
        wanted = {6: {-1: "left", 1: "right"}.get(frame.axes[6]),
                  7: {-1: "up", 1: "down"}.get(frame.axes[7])}
        events = []
        for axis, direction in wanted.items():
            hold = self._holds.get(axis)
            if hold and hold.direction != direction:
                reason = "direction_change" if direction else ("release" if frame.axes[axis] == 0 else "not_full")
                events.append(self._close(axis, now, timestamp, reason, True))
            if direction and axis not in self._holds:
                self._holds[axis] = _Hold(direction, now, timestamp, self._previous_fresh)
        self._previous_fresh = True

        safe = (isinstance(safety, SafetyState) and safety.armed is True
                and safety.software_manual is True and _number(safety.feedback_age_s)
                and 0 <= safety.feedback_age_s < .2)
        if not safe:
            self._reset_gate()
            return StepResult(Command(reason="disabled_or_stale"), tuple(events))
        if not frame.buttons[4]:
            self._released = True
            self._engaged = False
            return StepResult(Command(reason="l1_released"), tuple(events))
        if not self._released:
            return StepResult(Command(reason="release_l1_first"), tuple(events))
        x, y = frame.axes[6], frame.axes[7]
        if not self._engaged:
            if x != 0 or y != 0:
                return StepResult(Command(reason="center_dpad_first"), tuple(events))
            self._engaged = True
        if x != 0 and y != 0:
            return StepResult(Command(reason="diagonal_blocked"), tuple(events))
        direction = wanted[6] or wanted[7]
        speeds = {"up": (.2, 0.0), "down": (-.2, 0.0),
                  "left": (0.0, .2), "right": (0.0, -.2)}
        if direction is None:
            return StepResult(Command(reason="neutral" if x == y == 0 else "not_full"), tuple(events))
        vx, wz = speeds[direction]
        return StepResult(Command(vx, 0.0, wz, "enabled"), tuple(events))

    def finish(self, now, timestamp):
        if not _number(now) or (self._last_now is not None and now < self._last_now):
            now = self._last_now if self._last_now is not None else 0.0
        events = self._close_all(now, timestamp, "monitor_stopped")
        self._reset_gate()
        return StepResult(Command(reason="monitor_stopped"), events)
