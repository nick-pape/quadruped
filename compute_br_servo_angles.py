#!/usr/bin/env python3
"""
Compute servo angles for the back-right (BR) leg from a precomputed foot path.

- Input: a JSON with points [{"x": cm, "y": cm}] (no numpy/bezier).
- Output: a JSON with angles [{"shoulder": deg, "elbow": deg}] parallel to frames.
- Mirrors Quadruped.inverse_positioning kinematics and right-side mapping.

Usage:
  python3 compute_br_servo_angles.py \
    --in gaits/back_right_leg_standard.json \
    --out gaits/back_right_leg_standard_servo.json

Notes:
- No hardware dependency. Uses same constants as gait_logic/quadruped.py (right side offsets).
- If any frame is unreachable (c2 out of [-1,1]), the script will stop with a clear error.
"""
import argparse
import json
import math
import os
from typing import List, Dict, Any

# Match Quadruped constants
A1 = 10.0       # upper leg length (cm)
A2 = 10.5       # lower leg length (cm)
L = 2.0         # lateral offset used in y' (same as code)
# Right-side offsets (right=True) — per quadruped.py
SHOULDER_OFFSET_RIGHT = 10.0
ELBOW_OFFSET_RIGHT = 20.0


def rad_to_degree(rad: float) -> float:
    return rad * 180.0 / math.pi


def inverse_br_servo_angles(x: float, y: float, z: float = 0.0,
                            a1: float = A1, a2: float = A2,
                            shoulder_offset: float = SHOULDER_OFFSET_RIGHT,
                            elbow_offset: float = ELBOW_OFFSET_RIGHT) -> Dict[str, float]:
    # Replicate the Quadruped.inverse_positioning math for right side (right=True)
    y_prime = -math.sqrt((z + L) ** 2 + y ** 2)

    c2 = (x**2 + y_prime**2 - a1**2 - a2**2) / (2.0 * a1 * a2)
    if c2 < -1.0 or c2 > 1.0:
        raise ValueError(f"Unreachable point (x={x}, y={y}): c2={c2} outside [-1,1]")

    s2 = math.sqrt(1.0 - c2**2)
    theta2 = math.atan2(s2, c2)

    c2 = math.cos(theta2)
    s2 = math.sin(theta2)

    denom = (x**2 + y_prime**2)
    c1 = (x * (a1 + (a2 * c2)) + y_prime * (a2 * s2)) / denom
    s1 = (y_prime * (a1 + (a2 * c2)) - x * (a2 * s2)) / denom
    theta1 = math.atan2(s1, c1)

    theta_shoulder = -theta1
    theta_elbow = theta_shoulder - theta2

    # Right-side mapping (right=True) from Quadruped.inverse_positioning
    shoulder_servo = 180.0 - rad_to_degree(theta_shoulder) + shoulder_offset
    elbow_servo = 130.0 - rad_to_degree(theta_elbow) + elbow_offset

    return {"shoulder": shoulder_servo, "elbow": elbow_servo}


def load_points(path: str) -> List[Dict[str, float]]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Input gait file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.get("points")
    if not isinstance(points, list) or len(points) == 0:
        raise ValueError("Input JSON must contain a non-empty 'points' array")
    # Normalize floats
    for i, p in enumerate(points):
        if not (isinstance(p, dict) and "x" in p and "y" in p):
            raise ValueError(f"Invalid point at index {i}: expected object with 'x' and 'y'")
        p["x"] = float(p["x"])
        p["y"] = float(p["y"])
    return points


def build_output_metadata(input_meta: Dict[str, Any], frames_count: int) -> Dict[str, Any]:
    return {
        "source": input_meta.get("name", "unknown"),
        "description": "Servo angles for BR leg computed from foot positions",
        "frames": frames_count,
        "units": "degrees",
        "side": "BR",
        "mapping": {
            "shoulder": "180 - deg(theta_shoulder) + SHOULDER_OFFSET_RIGHT",
            "elbow": "130 - deg(theta_elbow) + ELBOW_OFFSET_RIGHT",
            "offsets": {
                "shoulder_offset_right": SHOULDER_OFFSET_RIGHT,
                "elbow_offset_right": ELBOW_OFFSET_RIGHT
            },
            "leg_lengths_cm": {"a1": A1, "a2": A2},
            "L": L
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Compute BR servo angles from precomputed foot path JSON")
    parser.add_argument("--in", dest="input_path", default=os.path.join("gaits", "back_right_leg_standard.json"),
                        help="Input JSON with 'points' [{x,y}] in cm")
    parser.add_argument("--out", dest="output_path", default=os.path.join("gaits", "back_right_leg_standard_servo.json"),
                        help="Output JSON with 'angles' [{shoulder,elbow}] in degrees")
    args = parser.parse_args()

    # Load input
    with open(args.input_path, "r", encoding="utf-8") as f:
        input_data = json.load(f)
    points = input_data.get("points")
    if points is None:
        raise ValueError("Input JSON missing 'points' array")
    points = load_points(args.input_path)

    # Compute angles
    angles: List[Dict[str, float]] = []
    for p in points:
        ang = inverse_br_servo_angles(p["x"], p["y"], z=0.0)
        angles.append(ang)

    # Build output
    meta_in = input_data.get("metadata", {})
    out = {
        "metadata": build_output_metadata(meta_in, frames_count=len(angles)),
        "angles": angles
    }

    # Write output
    os.makedirs(os.path.dirname(args.output_path), exist_ok=True)
    with open(args.output_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote {len(angles)} frames to {args.output_path}")


if __name__ == "__main__":
    main()
