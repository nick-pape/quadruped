#!/usr/bin/env python3
from time import sleep
from gait_logic.quadruped import Quadruped, Motor

def set_side(robot: Quadruped, side: str, shoulder: float, elbow: float):
    if side == "L":
        robot.set_angle(Motor.FL_SHOULDER, shoulder)
        robot.set_angle(Motor.FL_ELBOW, elbow)
        robot.set_angle(Motor.BL_SHOULDER, shoulder)
        robot.set_angle(Motor.BL_ELBOW, elbow)
    else:  # "R"
        robot.set_angle(Motor.FR_SHOULDER, shoulder)
        robot.set_angle(Motor.FR_ELBOW, elbow)
        robot.set_angle(Motor.BR_SHOULDER, shoulder)
        robot.set_angle(Motor.BR_ELBOW, elbow)

def main():
    robot = Quadruped()

    # keep hip orientation from your original script
    robot.set_angle(Motor.FR_HIP, 60)
    robot.set_angle(Motor.FL_HIP, 150)

    while True:
        # sit
        set_side(robot, "R", 0,   0)
        set_side(robot, "L", 180, 0)
        sleep(2)

        # stand
        set_side(robot, "R", 90, 0)
        set_side(robot, "L", 90, 0)
        sleep(2)

if __name__ == "__main__":
    main()
