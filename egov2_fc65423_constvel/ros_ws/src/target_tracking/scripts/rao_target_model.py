#!/usr/bin/env python3
"""Bicycle target state and a small kinodynamic Hybrid A* implementation."""

import heapq
import math
from dataclasses import dataclass


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_diff(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


@dataclass
class TargetState:
    # [rear-wheel x, rear-wheel y, yaw, speed, longitudinal accel,
    #  lateral accel, steering angle, curvature], matching Section 4.1.
    x: float
    y: float
    yaw: float
    speed: float
    longitudinal_accel: float
    lateral_accel: float
    steering: float
    curvature: float

    @classmethod
    def from_vector(cls, values):
        if len(values) != 8:
            raise ValueError("target state must contain 8 values")
        return cls(*[float(value) for value in values])

    def as_vector(self):
        return [self.x, self.y, self.yaw, self.speed, self.longitudinal_accel,
                self.lateral_accel, self.steering, self.curvature]


def propagate(state, steering, acceleration, dt, wheelbase, max_speed):
    """Integrate Eq. (46) with a constant motion primitive over dt."""
    steering = float(steering)
    speed = clamp(state.speed + float(acceleration) * dt, 0.0, max_speed)
    mid_speed = 0.5 * (state.speed + speed)
    curvature = math.tan(steering) / wheelbase
    yaw_rate = mid_speed * curvature
    mid_yaw = state.yaw + 0.5 * yaw_rate * dt
    return TargetState(
        state.x + mid_speed * math.cos(mid_yaw) * dt,
        state.y + mid_speed * math.sin(mid_yaw) * dt,
        angle_diff(state.yaw + yaw_rate * dt, 0.0),
        speed,
        float(acceleration),
        # Equation (46): dot(a_tgn) = v_tg^2 * kappa_tg.
        state.lateral_accel + mid_speed * mid_speed * curvature * dt,
        steering,
        curvature,
    )


def integrate_primitive(state, steering, acceleration, duration, integration_dt,
                        wheelbase, max_speed, validator=None):
    """Integrate one primitive with the same fixed steps used at execution."""
    result = TargetState.from_vector(state.as_vector())
    remaining = float(duration)
    while remaining > 1e-9:
        step_dt = min(float(integration_dt), remaining)
        result = propagate(result, steering, acceleration, step_dt, wheelbase, max_speed)
        if validator is not None and not validator(result):
            return None
        remaining -= step_dt
    return result


class HybridAStar:
    def __init__(self, scene, config, max_speed, control_rate_hz):
        self.scene = scene
        self.altitude = float(config["altitude_m"])
        self.wheelbase = float(config["wheelbase_m"])
        self.clearance = float(config["collision_clearance_m"])
        self.goal_tolerance = float(config["goal_tolerance_m"])
        self.steering_set = tuple(float(value) for value in config["steering_set_rad"])
        self.acceleration_set = tuple(float(value) for value in config["acceleration_set_mps2"])
        self.steering_cost_weight = float(config["steering_cost_weight"])
        self.acceleration_cost_weight = float(config["acceleration_cost_weight"])
        self.dt = float(config["action_dt_s"])
        self.integration_dt = 1.0 / float(control_rate_hz)
        self.max_speed = float(max_speed)
        self.max_expansions = int(config["max_expansions"])
        self.grid = float(config["grid_resolution_m"])
        self.yaw_bins = max(1, int(round(360.0 / float(config["yaw_resolution_deg"]))))
        self.speed_grid = float(config["speed_resolution_mps"])

    def _key(self, state):
        yaw = (state.yaw + math.pi) % (2.0 * math.pi) - math.pi
        return (round(state.x / self.grid), round(state.y / self.grid),
                int(round((yaw + math.pi) / (2.0 * math.pi) * self.yaw_bins) % self.yaw_bins),
                int(round(state.speed / self.speed_grid)))

    def _valid(self, state):
        margin = self.clearance
        if not (-self.scene.size_x / 2.0 + margin <= state.x <= self.scene.size_x / 2.0 - margin and
                -self.scene.size_y / 2.0 + margin <= state.y <= self.scene.size_y / 2.0 - margin):
            return False
        # Check the primitive endpoint and intermediate samples for a swept
        # collision, as required for a kinodynamic expansion.
        return not self.scene.point_collision(state.x, state.y, self.altitude, clearance=margin)

    def _expand(self, state, steering, acceleration):
        return integrate_primitive(
            state, steering, acceleration, self.dt, self.integration_dt,
            self.wheelbase, self.max_speed, validator=self._valid)

    def _has_progressing_successor(self, state):
        current_key = self._key(state)
        for steering in self.steering_set:
            for acceleration in self.acceleration_set:
                successor = self._expand(state, steering, acceleration)
                if successor is not None and self._key(successor) != current_key:
                    return True
        return False

    def plan(self, start, goal):
        start = TargetState.from_vector(start.as_vector())
        if not self._valid(start):
            return []
        queue = []
        serial = 0
        start_key = self._key(start)
        heapq.heappush(queue, (0.0, 0.0, serial, start_key, start, []))
        best = {start_key: 0.0}
        explored = 0
        while queue and explored < self.max_expansions:
            _, cost, _, key, state, actions = heapq.heappop(queue)
            if cost > best.get(key, float("inf")) + 1e-9:
                continue
            explored += 1
            distance = math.hypot(state.x - goal[0], state.y - goal[1])
            if distance <= self.goal_tolerance and self._has_progressing_successor(state):
                return actions
            for steering in self.steering_set:
                for acceleration in self.acceleration_set:
                    candidate = self._expand(state, steering, acceleration)
                    if candidate is None:
                        continue
                    candidate_key = self._key(candidate)
                    new_cost = (cost + self.dt + self.steering_cost_weight * abs(steering) +
                                self.acceleration_cost_weight * abs(acceleration))
                    if new_cost >= best.get(candidate_key, float("inf")):
                        continue
                    best[candidate_key] = new_cost
                    serial += 1
                    heuristic = math.hypot(candidate.x - goal[0], candidate.y - goal[1]) / max(self.max_speed, 0.1)
                    heapq.heappush(queue, (new_cost + heuristic, new_cost, serial, candidate_key,
                                           candidate, actions + [(steering, acceleration)]))
        return []
