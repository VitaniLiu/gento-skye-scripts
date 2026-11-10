"""Inference-local measured-path recorder and reverse base controller."""
from dataclasses import dataclass
import math


def wrap(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def clip(value, limit):
    return max(-limit, min(limit, value))


@dataclass
class Sample:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


class RecordedPath:
    def __init__(self):
        self.points = []

    def observe(self, sample):
        point = (sample.x, sample.y, sample.yaw)
        if not self.points or math.hypot(point[0] - self.points[-1][0], point[1] - self.points[-1][1]) >= .025 \
                or abs(wrap(point[2] - self.points[-1][2])) >= .075:
            self.points.append(point)

    def controller(self):
        if not self.points:
            raise RuntimeError("no chassis origin was recorded")
        return ReturnController(self.points)


class ReturnController:
    def __init__(self, points):
        self.points = list(points)
        self.index = len(self.points) - 1
        self.finished = False
        self.settled_at = None
        self.final_turn = False

    def step(self, sample, now):
        origin = self.points[0]
        target = self.points[max(0, self.index - 1)]
        end = self.points[self.index]
        dx, dy = target[0] - sample.x, target[1] - sample.y
        distance = math.hypot(dx, dy)
        segment_length = math.hypot(target[0] - end[0], target[1] - end[1])
        yaw_error = wrap(target[2] - sample.yaw)
        pivot = segment_length < .02 and abs(wrap(target[2] - end[2])) >= .035
        if self.index > 1 and pivot:
            if abs(yaw_error) > .035:
                return 0.0, clip(2 * yaw_error, .1)
            self.index -= 1
            return 0.0, 0.0
        if self.index > 1 and distance <= .015:
            self.index -= 1
            return 0.0, 0.0
        if self.index <= 1 and distance <= .007:
            self.final_turn = True
        if self.final_turn:
            yaw_error = wrap(origin[2] - sample.yaw)
            if abs(yaw_error) > math.radians(.5):
                self.settled_at = None
                return 0.0, clip(2 * yaw_error, .05)
            if abs(sample.vx) <= .01 and abs(sample.vy) <= .01 and abs(sample.wz) <= .02:
                self.settled_at = now if self.settled_at is None else self.settled_at
                if now - self.settled_at >= .5:
                    self.finished = True
            else:
                self.settled_at = None
            return 0.0, 0.0
        bearing = math.atan2(dy, dx)
        reverse = math.cos(bearing - target[2]) < 0
        steering = wrap(bearing + (math.pi if reverse else 0) - sample.yaw)
        speed = min(.05, 2 * distance)
        return (-speed if reverse else speed), clip(.7 * steering, .02)
