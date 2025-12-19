#!/usr/bin/env python3
"""
Zero-math back-left (BL) leg playback.

- Loads precomputed servo angles from a JSON file and applies them directly.
- No IK, numpy, or bezier; depends only on gait_logic.quadruped.Quadruped.

Usage:
  python3 test_bl_leg_angles.py --angles gaits/back_left_leg_standard_servo.json --hz 20 --loops 0

Notes:
- BL leg uses shoulder + elbow only; no hip servo for BL.
- Ctrl-C exits gracefully and returns to calibrate.
"""
import argparse
import json
import os
import time
from typing import List, Dict

from gait_logic.quadruped import Quadruped, Motor


def load_angles(path: str) -> List[Dict[str, float]]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Angles file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    angles = data.get("angles")
    if not isinstance(angles, list) or len(angles) == 0:
        raise ValueError("Angles JSON must contain a non-empty 'angles' array")
    for i, a in enumerate(angles):
        if not (isinstance(a, dict) and "shoulder" in a and "elbow" in a):
            raise ValueError(f"Invalid angle at index {i}: expected object with 'shoulder' and 'elbow'")
        a["shoulder"] = float(a["shoulder"])  # clamp later if needed
        a["elbow"] = float(a["elbow"])        # clamp later if needed
    return angles


def clamp(value: float, lo: float = 0.0, hi: float = 180.0) -> float:
    return max(lo, min(hi, value))


def run_bl_angles(robot: Quadruped, angles: List[Dict[str, float]], hz: float, loops: int, clamp_angles: bool) -> None:
    dt = 1.0 / max(hz, 1.0)
    cycle = 0
    try:
        while True:
            cycle += 1
            for a in angles:
                shoulder = a["shoulder"]
                elbow = a["elbow"]
                if clamp_angles:
                    shoulder = clamp(shoulder)
                    elbow = clamp(elbow)
                robot.set_angle(Motor.BL_SHOULDER, shoulder)
                robot.set_angle(Motor.BL_ELBOW, elbow)
                robot.set_angle(Motor.FL_SHOULDER, shoulder)
                robot.set_angle(Motor.FL_ELBOW, elbow)
                time.sleep(dt)
            if loops > 0 and cycle >= loops:
                break
    except KeyboardInterrupt:
        print("\nInterrupted. Returning to calibration.")
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="Back-left leg angle playback (zero math)")
    parser.add_argument("--angles", default=os.path.join("gaits", "back_left_leg_standard_servo.json"),
                        help="Path to angles JSON with 'angles' [{shoulder,elbow}] in degrees")
    parser.add_argument("--hz", type=float, default=20.0, help="Playback rate in Hz")
    parser.add_argument("--loops", type=int, default=0, help="Number of cycles to run (0=infinite)")
    parser.add_argument("--no-clamp", action="store_true", help="Do not clamp angles to [0,180]")
    args = parser.parse_args()

    angles = load_angles(args.angles)

    robot = Quadruped()
    robot.calibrate()
    time.sleep(0.5)
    robot.set_angle(Motor.FR_HIP, 60)
    robot.set_angle(Motor.FL_HIP, 150)
    print(f"Loaded {len(angles)} angle frames from {args.angles}. Starting BL angle playback at {args.hz} Hz...")

    run_bl_angles(robot, angles, hz=args.hz, loops=args.loops, clamp_angles=(not args.no_clamp))

    print("Done.")


if __name__ == "__main__":
    main()
