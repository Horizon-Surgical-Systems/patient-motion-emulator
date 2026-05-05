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

    # Open robot connection
    with mdr.Robot() as robot:
        robot.Connect(address=Parameter.ROBOT_IP_ADDRESS, disconnect_on_exception=False)
        robot.ActivateRobot()

        # Check if the robot is 6DOF
        if not robot.GetRobotInfo().num_joints == 6:
            print("Not a 6-axis robot. This program is designed for 6-axis robots, so some features may not work as expected.")
            sys.exit(0)

        # Set linear speed before moving
        robot.SetJointVelLimit(Parameter.MAX_JOINT_VEL_PERCENTAGE)

        # Prepare the robot for operation.
        robot.ActivateAndHome()
        robot.WaitHomed()
        print('Robot is homed and ready.')

        # Set robot TRF from FRF
        x_offset = Parameter.EYE_PUCK_OFFSET[0]
        y_offset = Parameter.EYE_PUCK_OFFSET[1]
        z_offset = Parameter.EYE_PUCK_OFFSET[2]
        robot.SetTrf(x_offset, y_offset, z_offset, 0, 0, 0)

        #  Move the robot to the initial pose defined by the user
        robot.MoveJoints(*Parameter.ROBOT_EYE_PUCK_INIT_POSE)
        print('Waiting for robot to finish moving to initial pose...')
        robot.WaitIdle(60)

        # Interactive control loop
        print("\n=== Robot Control Loop ===")
        print("Enter variable=value pairs to control the robot.")
        print("Supported variables: 'home', 'print', 'temporal', 'nasal', 'superior', 'inferior', 'cw', 'ccw'")
        print("Examples:")
        print("  move=home   - Perform homing")
        print("  move=init   - Move to initial pose")
        print("  print=1     - Print current robot pose")
        print("  temporal=10 - Perform temporal rotation = 10 deg")
        print("  nasal=10    - Perform nasal rotation = 10 deg")
        print("  exit=1      - Exit the control loop")
        print()

        while True:
            try:
                # Reset values
                ux_val = 0.0
                uy_val = 0.0
                uz_val = 0.0

                # Get user input
                user_input = input("Enter command (variable=value): ").strip()
                
                if not user_input:
                    continue
                
                # Parse the input
                if '=' in user_input:
                    variable, value = user_input.split('=', 1)
                    variable = variable.strip().lower()
                    value = value.strip()
                else:
                    print("Invalid format. Use variable=value format.")
                    continue

                # Impose a hard constraint at 15 deg for safety (only if value is a number)
                try:
                    val = float(value)
                    if abs(val) > 15:
                        print("Value exceeds 15 deg limit.")
                        continue
                except ValueError:
                    pass

                if variable == 'exit':
                    print("Exiting robot control loop...")
                    robot.DeactivateRobot()
                    break
                elif variable == 'move':
                    if value.lower() == 'home':
                        print('Moving robot to home position...')
                        robot.ActivateAndHome()
                        robot.WaitHomed()
                        print('Robot is at home.')
                    elif value.lower() == 'init':
                        print('Moving robot to initial pose...')
                        robot.MoveJoints(*Parameter.ROBOT_EYE_PUCK_INIT_POSE)
                        robot.WaitIdle(60)
                        print('Robot is at initial pose.')
                    else:
                        print(f"Unknown move command: {value}")

                elif variable == 'temporal':
                    try:
                        uy_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, 0, uy_val, 0)
                        print(f'Move {uy_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'nasal':
                    try:
                        uy_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, 0, -uy_val, 0)
                        print(f'Move {uy_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")

                elif variable == 'cw':
                    try:
                        uz_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, 0, 0, uz_val)
                        print(f'Move {uz_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'ccw':
                    try:
                        uz_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, 0, 0, -uz_val)
                        print(f'Move {uz_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")

                elif variable == 'superior':
                    try:
                        ux_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, ux_val, 0, 0)
                        print(f'Move {ux_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'inferior':
                    try:
                        ux_val = float(value)
                        robot.MoveLinRelTrf(0, 0, 0, -ux_val, 0, 0)
                        print(f'Move {ux_val} deg {variable} from current')
                    except ValueError:
                        print(f"Invalid value: {value}")

                elif variable == 'print':
                    if value == '1':
                        current_pose = robot.GetRtTargetJointPos()
                        print(f"Current robot joint angles: {current_pose}")
                    else:
                        print(f"Unknown print command: {value}")

                else:
                    print(f"Unknown variable: {variable}")

            except KeyboardInterrupt:
                print("\nInterrupted by user. Exiting control loop...")
                robot.DeactivateRobot()
                break
            except Exception as e:
                print(f"Error: {e}")

    # Exiting the "with" block automatically disconnects from the robot.
    print('Now disconnected from the robot.')