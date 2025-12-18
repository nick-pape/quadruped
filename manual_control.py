#!/usr/bin/env python3
"""
Simple manual control script for quadruped robot.
Usage: Enter commands in format "L,90,90" or "R,90,90"
where first value is L/R (side), second is shoulder angle, third is elbow angle.
"""

from gait_logic.quadruped import Quadruped, Motor

def main():
    print("Initializing quadruped...")
    robot = Quadruped()
    print("Robot initialized!")
    print("\nEnter commands in format: L,90,90 or R,90,90")
    print("  L/R: Left or Right side")
    print("  First number: Shoulder angle (0-270)")
    print("  Second number: Elbow angle (0-270)")
    print("Type 'quit' to exit\n")
    
    robot.set_angle(Motor.FR_HIP, 60)
    robot.set_angle(Motor.FL_HIP, 150)

    while True:
        try:
            command = input("Enter command: ").strip()
            
            if command.lower() in ['quit', 'exit', 'q']:
                print("Exiting...")
                break
            
            # Parse command
            parts = command.split(',')
            if len(parts) != 3:
                print("Error: Command must have 3 parts (e.g., L,90,90)")
                continue
            
            side = parts[0].strip().upper()
            shoulder_angle = float(parts[1].strip())
            elbow_angle = float(parts[2].strip())
            
            # Validate input
            if side not in ['L', 'R']:
                print("Error: Side must be 'L' or 'R'")
                continue
            
            if not (0 <= shoulder_angle <= 270):
                print("Error: Shoulder angle must be between 0 and 270")
                continue
            
            if not (0 <= elbow_angle <= 270):
                print("Error: Elbow angle must be between 0 and 270")
                continue
            
            # Set motor angles
            if side == 'L':
                robot.set_angle(Motor.FL_SHOULDER, shoulder_angle)
                robot.set_angle(Motor.FL_ELBOW, elbow_angle)
                robot.set_angle(Motor.BL_SHOULDER, shoulder_angle)
                robot.set_angle(Motor.BL_ELBOW, elbow_angle)
                print(f"Left side: Shoulder={shoulder_angle}°, Elbow={elbow_angle}°")
            else:
                robot.set_angle(Motor.FR_SHOULDER, shoulder_angle)
                robot.set_angle(Motor.FR_ELBOW, elbow_angle)
                robot.set_angle(Motor.BR_SHOULDER, shoulder_angle)
                robot.set_angle(Motor.BR_ELBOW, elbow_angle)
                print(f"Right side: Shoulder={shoulder_angle}°, Elbow={elbow_angle}°")
                
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except ValueError as e:
            print(f"Error: Invalid number format - {e}")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
