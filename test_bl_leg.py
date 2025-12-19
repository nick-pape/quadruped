#!/usr/bin/env python3
"""
On-the-fly gait generator using numpy + bezier curves.

- Generates foot positions dynamically (no JSON).
- Uses a single precomputed base gait profile with per-leg phase offsets.
- Drives all 4 legs in a stable crawl-style pattern using Quadruped.leg_position().

Usage:
  python3 test_bl_leg.py --hz 20 --loops 0 --cycle-seconds 1.5 --duty 0.80 --step-len 2.0 --step-height 1.0

Notes:
- Requires hardware access via adafruit_servokit in gait_logic.quadruped. Tested on Pi.
- `--loops 0` runs indefinitely; set a positive number to run that many cycles.
"""
import argparse
import time

import numpy as np
import bezier

from gait_logic.quadruped import Quadruped


def build_gait_profile(
    hz: float, 
    cycle_seconds: float, 
    duty: float, 
    step_len: float, 
    step_height: float, 
    x_center: float, 
    y_ground: float
) -> tuple:
    """
    Generate a single-leg gait profile using numpy + bezier curves.
    
    Args:
        hz: Update frequency (Hz)
        cycle_seconds: Duration of one gait cycle (seconds)
        duty: Stance fraction in [0.55, 0.95]
        step_len: Forward/back half-range (cm)
        step_height: Swing lift height (cm)
        x_center: Nominal x center (cm)
        y_ground: Nominal ground y position (cm)
    
    Returns:
        Tuple of (x_profile, y_profile) numpy arrays of length N
    """
    # Validate inputs
    if duty < 0.55 or duty > 0.95:
        raise ValueError(f"duty must be in [0.55, 0.95], got {duty}")
    if hz <= 0:
        raise ValueError(f"hz must be positive, got {hz}")
    if cycle_seconds <= 0:
        raise ValueError(f"cycle_seconds must be positive, got {cycle_seconds}")
    
    # Compute profile length
    N = max(4, int(round(hz * cycle_seconds)))
    stance_points = max(1, int(round(N * duty)))
    swing_points = max(1, N - stance_points)
    
    # Initialize arrays
    x_profile = np.zeros(N, dtype=np.float64)
    y_profile = np.zeros(N, dtype=np.float64)
    
    # Fill stance region [0:stance_points)
    # x moves linearly from +step_len to -step_len
    # y stays at y_ground
    if stance_points > 1:
        x_profile[:stance_points] = np.linspace(step_len, -step_len, stance_points, endpoint=False)
    else:
        x_profile[0] = step_len
    y_profile[:stance_points] = y_ground
    
    # Fill swing region [stance_points:N)
    # Use cubic Bezier curve for smooth lift and return
    # P0 = (-step_len, y_ground), P3 = (+step_len, y_ground)
    # P1, P2 lift to y_ground + step_height
    nodes = np.array([
        [-step_len, -step_len * 0.33, step_len * 0.33, step_len],  # x coords
        [y_ground, y_ground + step_height, y_ground + step_height, y_ground]  # y coords
    ], dtype=np.float64)
    
    curve = bezier.Curve(nodes, degree=3)
    
    # Sample swing curve
    if swing_points >= 1:
        u_vals = np.linspace(0, 1, swing_points, endpoint=True)
        swing_points_eval = curve.evaluate_multi(u_vals)
        x_profile[stance_points:N] = swing_points_eval[0, :swing_points]
        y_profile[stance_points:N] = swing_points_eval[1, :swing_points]
    
    return x_profile, y_profile


def run_gait(
    robot: Quadruped, 
    x_profile: np.ndarray, 
    y_profile: np.ndarray, 
    hz: float, 
    loops: int, 
    y_offset: float,
    phase_offsets: dict,
    x_center: float
) -> None:
    """
    Execute the gait using precomputed profiles and per-leg phase offsets.
    
    Args:
        robot: Quadruped instance
        x_profile: Precomputed x positions (length N)
        y_profile: Precomputed y positions (length N)
        hz: Update frequency (Hz)
        loops: Number of cycles to run (0 = infinite)
        y_offset: Global y offset (cm)
        phase_offsets: Dict mapping leg names to phase fractions (e.g., {'FR': 0.0, 'RL': 0.25, ...})
        x_center: Nominal x center (cm)
    """
    N = len(x_profile)
    dt = 1.0 / max(hz, 1.0)
    cycle = 0
    
    try:
        while True:
            cycle += 1
            for k in range(N):
                # Apply phase offsets to each leg
                for leg, phase_offset in phase_offsets.items():
                    idx = (k + int(round(phase_offset * N))) % N
                    x = x_profile[idx] + x_center
                    y = y_profile[idx] + y_offset
                    robot.leg_position(leg, x, y)
                
                time.sleep(dt)
            
            if loops > 0 and cycle >= loops:
                break
    except KeyboardInterrupt:
        print("\nInterrupted. Returning to calibration.")
        robot.calibrate()
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="On-the-fly gait generator using numpy + bezier")
    parser.add_argument("--hz", type=float, default=20.0, 
                        help="Update rate in Hz")
    parser.add_argument("--loops", type=int, default=0, 
                        help="Number of cycles to run (0=infinite)")
    parser.add_argument("--cycle-seconds", type=float, default=1.5, 
                        help="Gait cycle duration in seconds")
    parser.add_argument("--duty", type=float, default=0.80, 
                        help="Stance fraction in [0.55, 0.95]")
    parser.add_argument("--step-len", type=float, default=2.0, 
                        help="Forward/back half-range in cm")
    parser.add_argument("--step-height", type=float, default=2.0, 
                        help="Swing lift in cm")
    parser.add_argument("--x-center", type=float, default=0.0, 
                        help="Nominal x center in cm")
    parser.add_argument("--y-ground", type=float, default=-16.0, 
                        help="Nominal ground y in cm")
    parser.add_argument("--y-offset", type=float, default=0.0, 
                        help="Additional global y offset in cm")
    args = parser.parse_args()

    # Build gait profile
    x_profile, y_profile = build_gait_profile(
        hz=args.hz,
        cycle_seconds=args.cycle_seconds,
        duty=args.duty,
        step_len=args.step_len,
        step_height=args.step_height,
        x_center=args.x_center,
        y_ground=args.y_ground
    )
    
    N = len(x_profile)
    
    # Define per-leg phase offsets (crawl/wave gait)
    # FR=0.00, BR=0.25, FL=0.50, BL=0.75 ensures only one leg in swing at a time
    phase_offsets = {
        'FR': 0.00,
        'BR': 0.25,
        'FL': 0.50,
        'BL': 0.75
    }
    
    # Initialize robot and calibrate
    robot = Quadruped()
    robot.calibrate()
    time.sleep(0.5)
    
    # Print summary
    print(f"Gait profile: {N} points")
    print(f"  Hz: {args.hz}, Cycle: {args.cycle_seconds}s, Duty: {args.duty:.2f}")
    print(f"  Step length: {args.step_len} cm, Step height: {args.step_height} cm")
    print(f"  X center: {args.x_center} cm, Y ground: {args.y_ground} cm, Y offset: {args.y_offset} cm")
    print(f"  Phase offsets (crawl gait): {phase_offsets}")
    print(f"Starting gait (loops={'infinite' if args.loops == 0 else args.loops})...\n")

    run_gait(robot, x_profile, y_profile, args.hz, args.loops, args.y_offset, phase_offsets, args.x_center)

    print("Done.")


if __name__ == "__main__":
    main()
