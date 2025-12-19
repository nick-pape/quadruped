#!/usr/bin/env python3
"""
Back-left leg gait test script.

- Loads precomputed foot positions from a JSON file (no numpy/bezier).
- Drives the BL leg using Quadruped.inverse_positioning via `leg_position('BL', x, y)`.
- Default gait file: gaits/back_left_leg_standard.json

Usage:
  python3 test_bl_leg.py --gait gaits/back_left_leg_standard.json --hz 20 --loops 0 --y-offset 0

Notes:
- Requires hardware access via adafruit_servokit in gait_logic.quadruped. Tested on Pi.
- `--loops 0` runs indefinitely; set a positive number to run that many cycles.
"""
import argparse
import json
import os
import time
from typing import List, Dict

from gait_logic.quadruped import Quadruped


def load_gait_points(path: str) -> List[Dict[str, float]]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Gait file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.get("points")
    if not isinstance(points, list) or len(points) == 0:
        raise ValueError("Gait JSON must contain a non-empty 'points' array")
    # Validate shape
    for i, p in enumerate(points):
        if not (isinstance(p, dict) and "x" in p and "y" in p):
            raise ValueError(f"Invalid point at index {i}: expected object with 'x' and 'y'")
        # Coerce to float
        p["x"] = float(p["x"])
        p["y"] = float(p["y"])
    return points


def run_bl_gait(robot: Quadruped, points: List[Dict[str, float]], hz: float, loops: int, y_offset: float) -> None:
    dt = 1.0 / max(hz, 1.0)
    cycle = 0
    try:
        while True:
            cycle += 1
            for p in points:
                x = p["x"]
                y = p["y"] + y_offset
                # BL leg has no hip in inverse_positioning call
                robot.leg_position('BL', x, y)
                time.sleep(dt)
            if loops > 0 and cycle >= loops:
                break
    except KeyboardInterrupt:
        print("\nInterrupted. Returning to calibration.")
        robot.calibrate()
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="Back-left leg gait test (precomputed JSON)")
    parser.add_argument("--gait", default=os.path.join("gaits", "back_left_leg_standard.json"),
                        help="Path to gait JSON with 'points' [{x,y},...]")
    parser.add_argument("--hz", type=float, default=20.0, help="Update rate in Hz")
    parser.add_argument("--loops", type=int, default=0, help="Number of cycles to run (0=infinite)")
    parser.add_argument("--y-offset", type=float, default=0.0, help="Add a constant offset to y (cm)")
    args = parser.parse_args()

    points = load_gait_points(args.gait)

    robot = Quadruped()
    # Safe starting pose
    robot.calibrate()
    time.sleep(0.5)
    print(f"Loaded {len(points)} points from {args.gait}. Starting BL gait at {args.hz} Hz...")

    run_bl_gait(robot, points, hz=args.hz, loops=args.loops, y_offset=args.y_offset)

    print("Done.")


if __name__ == "__main__":
    main()
