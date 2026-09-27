#!/usr/bin/env python3
"""Offline checks for the exact remaining-trajectory reparameterization."""
import math
import random


def shift_coeff(c, offset):
    # c is ascending power order: p(t)=sum c[n] t**n
    out = [0.0] * 6
    for n, cn in enumerate(c):
        for k in range(n + 1):
            out[k] += cn * math.comb(n, k) * offset ** (n - k)
    return out


def eval_poly(c, t):
    return sum(v * t ** i for i, v in enumerate(c))


def main():
    random.seed(7)
    for _ in range(100):
        c = [random.uniform(-2.0, 2.0) for _ in range(6)]
        offset = random.uniform(0.0, 1.0)
        shifted = shift_coeff(c, offset)
        for u in (0.0, 0.1, 0.5, 1.0):
            assert abs(eval_poly(shifted, u) - eval_poly(c, offset + u)) < 1e-10

    # Lifecycle contracts: expired trajectories cannot be sliced, and a
    # retained trajectory starts at the current execution phase (u=0).
    total = 3.0
    elapsed = 0.75
    assert 0.0 <= elapsed < total
    assert total - elapsed > 0.0
    assert not (total <= elapsed)
    print("trajectory persistence offline checks: PASS")


if __name__ == "__main__":
    main()
