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


def run_bl_angles(
    robot: Quadruped,
    bl_angles: List[Dict[str, float]],
    hz: float,
    loops: int,
    clamp_angles: bool,
    right_shoulder: float,
    right_elbow: float,
    br_angles: List[Dict[str, float]] = None,
) -> None:
    dt = 1.0 / max(hz, 1.0)
    cycle = 0
    try:
        while True:
            cycle += 1
            # Iterate by index to optionally sync BR angles per frame
            frame_count = len(bl_angles)
            for i in range(frame_count):
                a_bl = bl_angles[i]
                shoulder = a_bl["shoulder"]
                elbow = a_bl["elbow"]
                if clamp_angles:
                    shoulder = clamp(shoulder)
                    elbow = clamp(elbow)
                robot.set_angle(Motor.BL_SHOULDER, shoulder)
                robot.set_angle(Motor.BL_ELBOW, elbow)

                if br_angles:
                    a_br = br_angles[i % len(br_angles)]
                    r_sh = clamp(a_br["shoulder"]) if clamp_angles else a_br["shoulder"]
                    r_el = clamp(a_br["elbow"]) if clamp_angles else a_br["elbow"]
                else:
                    # Stabilize right side (both front and back) to ideal angles
                    r_sh = clamp(right_shoulder) if clamp_angles else right_shoulder
                    r_el = clamp(right_elbow) if clamp_angles else right_elbow
                robot.set_angle(Motor.FR_SHOULDER, r_sh)
                robot.set_angle(Motor.FR_ELBOW, r_el)
                robot.set_angle(Motor.BR_SHOULDER, r_sh)
                robot.set_angle(Motor.BR_ELBOW, r_el)
                time.sleep(dt)
            if loops > 0 and cycle >= loops:
                break
    except KeyboardInterrupt:
        print("\nInterrupted. Returning to calibration.")
        robot.calibrate()
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="Back-left angle playback (zero math) with optional right-side gait or stabilization")
    parser.add_argument("--angles", default=os.path.join("gaits", "back_left_leg_standard_servo.json"),
                        help="Path to angles JSON with 'angles' [{shoulder,elbow}] in degrees")
    parser.add_argument("--hz", type=float, default=20.0, help="Playback rate in Hz")
    parser.add_argument("--loops", type=int, default=0, help="Number of cycles to run (0=infinite)")
    parser.add_argument("--no-clamp", action="store_true", help="Do not clamp angles to [0,180]")
    parser.add_argument("--right-angles", default=None, help="Path to BR angles JSON [{shoulder,elbow}] to move right side with gait")
    parser.add_argument("--right-shoulder", type=float, default=90.0, help="Ideal right-side shoulder angle (deg) when no --right-angles")
    parser.add_argument("--right-elbow", type=float, default=0.0, help="Ideal right-side elbow angle (deg) when no --right-angles")
    args = parser.parse_args()

    angles_bl = load_angles(args.angles)
    angles_br = load_angles(args.right_angles) if args.right_angles else None

    robot = Quadruped()
    robot.calibrate()
    time.sleep(0.5)
    robot.set_angle(Motor.FR_HIP, 60)
    robot.set_angle(Motor.FL_HIP, 150)
    print(f"Loaded {len(angles_bl)} BL frames from {args.angles}.")
    if angles_br:
        print(f"Loaded {len(angles_br)} BR frames from {args.right_angles}. Moving both sides.")
    else:
        print(f"No BR angles provided; stabilizing right side at shoulder={args.right_shoulder}°, elbow={args.right_elbow}°")

    run_bl_angles(
        robot,
        angles_bl,
        hz=args.hz,
        loops=args.loops,
        clamp_angles=(not args.no_clamp),
        right_shoulder=args.right_shoulder,
        right_elbow=args.right_elbow,
        br_angles=angles_br,
    )

    print("Done.")


if __name__ == "__main__":
    main()
