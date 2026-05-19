import mecademicpy.robot as mdr
import numpy as np
import os
import sys
import time
import argparse
import warnings
import Parameter
from TrajectoryPlanner import TrajectoryPlanner
from TrajectoryUtils import TrajectoryUtils

if __name__ == "__main__":

    # Create the mutually exclusive group
    parser = argparse.ArgumentParser()
    parser.add_argument("--head", action='store_true', help="Generate head motion")
    parser.add_argument("--eye", action='store_true', help="Generate eye motion")
    args = parser.parse_args()

    # Open robot connection
    with mdr.Robot() as robot:
        robot.Connect(address=Parameter.ROBOT_IP_ADDRESS, disconnect_on_exception=False)

        robot.ActivateRobot()

        # Check if the robot is 6DOF
        if not robot.GetRobotInfo().num_joints == 6:
            print("Not a 6-axis robot. This program is designed for 6-axis robots, so some features may not work as expected.")
            sys.exit(0)

        # Set linear speed before moving
        robot.SetCartLinVel(1) # mm/s for initial speed
        robot.SetJointVelLimit(Parameter.MAX_JOINT_VEL_PERCENTAGE)
        robot.SetJointVel(1)
        robot.ResetError()
        print('Set initial linear speed to 1 mm/s')

        # Prepare the robot for operation.
        robot.ActivateAndHome()
        robot.WaitHomed()
        print('Robot is homed and ready.')

        #  Move the robot to the initial pose defined by the user
        robot.MoveJoints(*Parameter.ROBOT_HEAD_INIT_POSE)
        print('Waiting for robot to finish moving to initial pose...')
        robot.WaitIdle(60)

        # Set robot TRF from FRF
        robot.SetTrf(0, 0, 0, 0, 0, 15)

        x_offset = Parameter.HEAD_OFFSET[0]
        y_offset = Parameter.HEAD_OFFSET[1]
        z_offset = Parameter.HEAD_OFFSET[2]
        robot.SetTrf(0, 6, 0, 0, 0, 15)
        # robot.SetTrf(x_offset, y_offset, z_offset, 0, 0, 0)

        x_rot = 10
        y_rot = 10
        z_rot = 10

        while True:
            try:               
                robot.MoveLinRelTrf(0, 0, 0, 0, 0, z_rot)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, 0, -z_rot)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, 0, -z_rot)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, 0, z_rot)
                robot.WaitIdle(60)

                robot.Delay(2) 

                robot.MoveLinRelTrf(0, 0, 0, 0, y_rot, 0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, -y_rot, -0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, -y_rot, -0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, 0, y_rot, 0)
                robot.WaitIdle(60)

                robot.Delay(2)

                robot.MoveLinRelTrf(0, 0, 0, x_rot, 0, 0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, -x_rot, 0, 0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, -x_rot, 0, 0)
                robot.WaitIdle(60)
                robot.MoveLinRelTrf(0, 0, 0, x_rot, 0, 0)
                robot.WaitIdle(60)

                robot.Delay(2)

            except KeyboardInterrupt:
                print("\nInterrupted by user. Exiting control loop...")
                robot.WaitIdle(60)
                robot.DeactivateRobot()
                break
            except Exception as e:
                print(f"Error: {e}")

    # Exiting the "with" block automatically disconnects from the robot.
    robot.DeactivateRobot()
    print('Now disconnected from the robot.')