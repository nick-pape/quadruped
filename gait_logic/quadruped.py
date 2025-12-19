from adafruit_servokit import ServoKit
import math
#import bezier
#import numpy as np
from typing import Callable

class Motor:
    """Represents a motor with a board address and channel number"""
    def __init__(self, board_address: int, channel: int):
        self.board_address = board_address
        self.channel = channel

    # Board 1 (0x40) motors
    FR_HIP: "Motor"
    BR_ELBOW: "Motor"
    FR_SHOULDER: "Motor"
    BR_SHOULDER: "Motor"
    FR_ELBOW: "Motor"

    # Board 2 (0x41) motors
    BL_SHOULDER: "Motor"
    BL_ELBOW: "Motor"
    FL_HIP: "Motor"
    FL_SHOULDER: "Motor"
    FL_ELBOW: "Motor"

# Initialize motor constants
# Board 1 (0x40)
Motor.FR_HIP = Motor(0x40, 4)
Motor.BR_ELBOW = Motor(0x40, 7)
Motor.FR_SHOULDER = Motor(0x40, 8)
Motor.BR_SHOULDER = Motor(0x40, 14)
Motor.FR_ELBOW = Motor(0x40, 15)
# Board 2 (0x41)
Motor.BL_SHOULDER = Motor(0x60, 0)
Motor.BL_ELBOW = Motor(0x60, 4)
Motor.FL_HIP = Motor(0x60, 8)
Motor.FL_SHOULDER = Motor(0x60, 14)
Motor.FL_ELBOW = Motor(0x60, 15)

class Quadruped:
    def __init__(self):
        # Initialize two servo boards
        self.kits = {
            0x40: ServoKit(channels=16, address=0x40),
            0x60: ServoKit(channels=16, address=0x60)
        }
        self.upper_leg_length = 10
        self.lower_leg_length = 10.5
        
        # Configure all motors
        motors = [
            Motor.FR_SHOULDER, Motor.FR_ELBOW, Motor.FR_HIP,
            Motor.FL_SHOULDER, Motor.FL_ELBOW, Motor.FL_HIP,
            Motor.BR_SHOULDER, Motor.BR_ELBOW,
            Motor.BL_SHOULDER, Motor.BL_ELBOW
        ]
        for motor in motors:
            self.kits[motor.board_address].servo[motor.channel].set_pulse_width_range(500, 2500)

    def set_angle(self, motor, degrees):
        """
        set the angle of a specific motor to a given angle
        :param motor: the Motor object
        :param degrees: the angle to put the motor to
        :returns: void
        """
        self.kits[motor.board_address].servo[motor.channel].angle = degrees

    def rad_to_degree(self,rad):
        """
        Converts radians to degrees
        :param rad: radians
        :returns: the corresponding degrees as a float
        """
        return rad*180/math.pi

    def calibrate(self):
        """
        sets the robot into the default "middle position" use this for attaching legs in right location
        :returns: void
        """
        #self.set_angle(Motor.FR_SHOULDER, 60)
        #self.set_angle(Motor.FR_ELBOW, 90)
        #self.set_angle(Motor.FR_HIP, 90)
        self.set_angle(Motor.FL_SHOULDER, 120)
        self.set_angle(Motor.FL_ELBOW, 90)
        self.set_angle(Motor.FL_HIP, 90)
        #self.set_angle(Motor.BR_SHOULDER, 60)
        #self.set_angle(Motor.BR_ELBOW, 90)
        self.set_angle(Motor.BL_SHOULDER, 120)
        self.set_angle(Motor.BL_ELBOW, 90)    
        
        #self.set_angle(Motor.FR_SHOULDER, 0)
        #self.set_angle(Motor.FR_ELBOW, 0)
        # self.set_angle(Motor.FR_HIP, 60)
        # self.set_angle(Motor.FL_SHOULDER, 180)
        # self.set_angle(Motor.FL_ELBOW, 0)
        # self.set_angle(Motor.FL_HIP, 150)
        # self.set_angle(Motor.BR_SHOULDER, 0)
        # self.set_angle(Motor.BR_ELBOW, 0)
        # self.set_angle(Motor.BL_SHOULDER, 180)
        # self.set_angle(Motor.BL_ELBOW, 0)  

    def inverse_positioning(self, shoulder, elbow, x, y, z=0, hip=None, right=True):
        '''
        Positions the end effector at a given position based on cartesian coordinates in 
        centimeter units and with respect to the should motor of the
        :param shoulder: motor id used for the shoulder
        :param elbow: motor id used for the elbow
        :param x: cartesian x with respect to shoulder motor (forward/back)
        :param y: cartesian y with respect to shoulder motor (up/down)
        :param z: cartesian z with respect to shoulder motor (left/right)
        :param hip: motor id used for the hip
        :param right: a boolean that flips the logic for left and right side to properly map "forward direction"
        :return: a list containing the appropriate angle for the shoulder and elbow
        '''
        L=2
        y_prime = -math.sqrt((z+L)**2 + y**2)
        thetaz = math.atan2(z+L,abs(y))-math.atan2(L,abs(y_prime))

        # Side-specific servo offsets: keep right as-is, tune left to better
        # align IK mid-pose with calibrate() for BL/FL.
        if right:
            shoulder_offset = 10
            elbow_offset = 20
        else:
            shoulder_offset = 16
            elbow_offset = 28
        a1 = self.upper_leg_length
        a2 = self.lower_leg_length

        c2 = (x**2+y_prime**2-a1**2-a2**2)/(2*a1*a2)
        s2 = math.sqrt(1-c2**2)
        theta2 = math.atan2(s2,c2)
        c2 = math.cos(theta2)
        s2 = math.sin(theta2)

        c1 = (x*(a1+(a2*c2)) + y_prime*(a2*s2))/(x**2+y_prime**2)
        s1 = (y_prime*(a1+(a2*c2)) - x*(a2*s2))/(x**2+y_prime**2)
        theta1 = math.atan2(s1,c1)
        # generate positions with respect to robot motors
        theta_shoulder = -theta1
        theta_elbow = theta_shoulder - theta2
        theta_hip = 0
        if right:
            theta_shoulder = 180 - self.rad_to_degree(theta_shoulder) + shoulder_offset
            theta_elbow = 130 - self.rad_to_degree(theta_elbow) + elbow_offset
            if hip:
                theta_hip = 90 - self.rad_to_degree(thetaz)
        else:
            theta_shoulder = self.rad_to_degree(theta_shoulder) - shoulder_offset
            theta_elbow = 50 + self.rad_to_degree(theta_elbow) - elbow_offset
            if hip:
                theta_hip = 90 + self.rad_to_degree(thetaz)
        self.set_angle(shoulder, theta_shoulder)
        self.set_angle(elbow, theta_elbow)
        if hip:
            self.set_angle(hip, theta_hip)
        # print("theta shoulder:",theta_shoulder,"\ttheta_elbow:",theta_elbow)
        return [theta_shoulder, theta_elbow]

    def leg_position(self, leg_id, x, y, z=0):
        """
        wrapper for inverse position that makes it easier to control each leg for making fixed paths
        :param led_id: string for the leg to be manipulated
        :param x: cartesian x with respect to shoulder motor (forward/back)
        :param y: cartesian y with respect to shoulder motor (up/down)
        :param z: cartesian z with respect to shoulder motor (left/right)
        """
        if leg_id == 'FL':
            self.inverse_positioning(Motor.FL_SHOULDER, Motor.FL_ELBOW, x, y, z=z, hip=Motor.FL_HIP, right=False)
        if leg_id == 'FR':
            self.inverse_positioning(Motor.FR_SHOULDER, Motor.FR_ELBOW, x, y, z=z, hip=Motor.FR_HIP, right=True)
        if leg_id == 'BL':
            self.inverse_positioning(Motor.BL_SHOULDER, Motor.BL_ELBOW, x, y, right=False)
        if leg_id == 'BR':
            self.inverse_positioning(Motor.BR_SHOULDER, Motor.BR_ELBOW, x, y, right=True)
    
    def move(self, controller = None):
        """
        Walks around based on the controller inputted momentum
        :param controller: the controller that is called to determine the robot momentum
        :returns: None, enters an infinite loop 
        """
        if controller is None:
            raise ValueError("Controller must be provided")
        momentum = np.asarray([0,0,1,0],dtype=np.float32)
        index = 0
        
        # Generate footstep
        s_vals = np.linspace(0.0, 1.0, 20)
        step_nodes = np.asfortranarray([
            [-1.0, -1.0, 1.0, 1.0],
            [-1.0, -1.0, 1.0, 1.0],
            [-15.0, -10, -10, -15.0],
        ])
        curve = bezier.Curve(step_nodes, degree=3)
        step = curve.evaluate_multi(s_vals)
        slide_nodes = np.asfortranarray([
            [1.0, -1.0],
            [1.0, -1.0],
            [-15.0, -15],
        ])
        curve = bezier.Curve(slide_nodes, degree=1)
        slide = curve.evaluate_multi(s_vals)

        motion = np.concatenate((step,slide), axis=1)

        close = False
        while not close:
            momentum = controller(momentum)
            tragectory = motion * momentum[:3, None]
            if momentum[3]:
                close = True
            x,z,y = tragectory
            # 
            i1 = index%40
            i2 = (index+20)%40 
            # Apply movement based movement
            self.inverse_positioning(Motor.FR_SHOULDER,Motor.FR_ELBOW,x[i1],y[i1]-1,z=z[i1],hip=Motor.FR_HIP,right=True)
            self.inverse_positioning(Motor.BR_SHOULDER,Motor.BR_ELBOW,x[i2],y[i2]+2,right=True)
            self.inverse_positioning(Motor.FL_SHOULDER,Motor.FL_ELBOW,x[i2],y[i2]-1,z=-z[i2],hip=Motor.FL_HIP,right=False)
            self.inverse_positioning(Motor.BL_SHOULDER,Motor.BL_ELBOW,x[i1],y[i1]+2,right=False)
            index += 1

