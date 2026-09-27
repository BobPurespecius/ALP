#!/usr/bin/env python3
"""Offline contract checks for three-class selection and repair timing."""

import math


ABSOLUTE_SAFE = "ABSOLUTE_SAFE"
IMPROVED_ONLY = "IMPROVED_ONLY"
INVALID = "INVALID"


def classify(kind, success, risk_valid, checks_passed, distance,
             nominal_distance, absolute_threshold=1.1, improvement=0.10):
    if not success or not risk_valid or not checks_passed:
        return INVALID
    if distance >= absolute_threshold:
        return ABSOLUTE_SAFE
    if kind != "NOMINAL" and distance >= nominal_distance + improvement:
        return IMPROVED_ONLY
    return INVALID


def allocate(points, max_vel=3.0, max_acc=6.0, eta_v=0.8, eta_a=0.8,
             speed_floor=0.10, min_piece_t=1.0e-3):
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    refs = [max(speed_floor, eta_v * max_vel) for _ in points]
    max_curvature = 0.0
    for i in range(1, len(points) - 1):
        incoming = tuple(points[i][j] - points[i - 1][j] for j in range(3))
        outgoing = tuple(points[i + 1][j] - points[i][j] for j in range(3))
        li = math.sqrt(sum(v * v for v in incoming))
        lo = math.sqrt(sum(v * v for v in outgoing))
        if li <= 1.0e-6 or lo <= 1.0e-6:
            continue
        cosine = max(-1.0, min(1.0,
            sum(a * b for a, b in zip(incoming, outgoing)) / (li * lo)))
        theta = math.acos(cosine)
        curvature = 2.0 * math.sin(0.5 * theta) / max(0.5 * (li + lo), 1.0e-6)
        max_curvature = max(max_curvature, curvature)
        refs[i] = max(speed_floor, min(refs[i],
            math.sqrt(eta_a * max_acc / (curvature + 1.0e-6))))
    durations = []
    for i, length in enumerate(lengths):
        speed = max(speed_floor, min(refs[i], refs[i + 1]))
        durations.append(max(min_piece_t, length / speed))
    return lengths, refs, durations, max_curvature


def main():
    assert classify("NOMINAL", True, True, True, 1.10, 1.10) == ABSOLUTE_SAFE
    assert classify("SIDE_PLUS", True, True, True, 0.95, 0.80) == IMPROVED_ONLY
    assert classify("SIDE_MINUS", True, True, True, 0.89, 0.80) == INVALID
    assert classify("SIDE_MINUS", True, False, True, 2.0, 0.80) == INVALID

    # Equal point count does not imply equal time: long segments receive more
    # time, and the segments adjacent to a sharp bend receive lower speed.
    points = [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
              (2.2, 0.0, 0.0), (2.2, 0.2, 0.0),
              (5.848, 0.2, 0.0)]
    lengths, refs, durations, max_curvature = allocate(points)
    assert all(math.isfinite(t) and t > 0.0 for t in durations)
    assert durations[1] > durations[0]
    assert refs[2] < 0.8 * 3.0
    assert max_curvature > 0.0
    assert sum(durations) > 0.667

    # Adding short guide points along a straight segment must not collapse the
    # total duration: arc length, not point count, controls timing.
    straight_sparse = [(0.0, 0.0, 0.0), (5.848, 0.0, 0.0)]
    straight_dense = [(5.848 * i / 24.0, 0.0, 0.0) for i in range(25)]
    sparse_total = sum(allocate(straight_sparse)[2])
    dense_total = sum(allocate(straight_dense)[2])
    assert abs(sparse_total - dense_total) < 1.0e-10

    print("three-class and arc-length timing offline checks: PASS")
    print("pathological_sample_duration=%.6f old_window=0.667000 "
          "min_piece_T=%.6f max_piece_T=%.6f max_curvature=%.6f" %
          (sum(durations), min(durations), max(durations), max_curvature))


if __name__ == "__main__":
    main()
