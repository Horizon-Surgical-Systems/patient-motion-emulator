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

        # Check if the robot is 6DOF
        if not robot.GetRobotInfo().num_joints == 6:
            print("Not a 6-axis robot. This program is designed for 6-axis robots, so some features may not work as expected.")
            sys.exit(0)
        
        # Prepare the robot for operation.
        robot.ActivateAndHome()
        robot.WaitHomed()
        print('Robot is homed and ready.')

        # Set robot TRF from FRF
        x_offset = 0.0
        y_offset = 0.0
        z_offset = 0.0
        robot.SetTrf(x_offset, y_offset, z_offset, 0, 0, 0)

        # Set linear speed
        robot.SetCartLinVel(Parameter.MAX_VELOCITY)

        #  Move the robot to the initial pose defined by the user
        init_pos = [0, 0, 30, 0, -30, 0]
        robot.MoveJoints(*init_pos)
        print('Waiting for robot to finish moving to initial pose...')
        robot.WaitIdle(60)

        # Interactive control loop
        print("\n=== Robot Control Loop ===")
        print("Enter variable=value pairs to control the robot.")
        print("Supported variables: 'home', 'print', 'temporal', 'nasal', 'superior', 'inferior', 'upward', 'downward'")
        print("Examples:")
        print("  move=home  - Perform homing")
        print("  move=init  - Move to initial pose")
        print("  print=1    - Print current robot pose")
        print("  temporal=1 - Perform temporal translation = +1 mm")
        print("  nasal=-2   - Perform nasal translation = -2 mm")
        print("  exit=1     - Exit the control loop")
        print()

        while True:
            try:
                # Reset values
                x_val = 0.0
                y_val = 0.0
                z_val = 0.0


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

                if variable == 'exit':
                    print("Exiting robot control loop...")
                    break

                elif variable == 'move':
                    if value.lower() == 'home':
                        print('Moving robot to home position...')
                        robot.ActivateAndHome()
                        robot.WaitHomed()
                        print('Robot is at home.')
                    elif value.lower() == 'init':
                        print('Moving robot to initial pose...')
                        robot.MoveJoints(*init_pos)
                        robot.WaitIdle(60)
                        print('Robot is at initial pose.')
                    else:
                        print(f"Unknown move command: {value}")

                elif variable == 'temporal':
                    try:
                        y_val = float(value)
                        robot.MoveLinRelTrf(0, y_val, 0, 0, 0, 0)
                        print(f'Move {y_val} mm {variable} to current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'nasal':
                    try:
                        y_val = float(value)
                        robot.MoveLinRelTrf(0, -y_val, 0, 0, 0, 0)
                        print(f'Move {y_val} mm {variable} to current')
                    except ValueError:
                        print(f"Invalid value: {value}")

                elif variable == 'superior':
                    try:
                        z_val = float(value)
                        robot.MoveLinRelTrf(0, 0, z_val, 0, 0, 0)
                        print(f'Move {z_val} mm {variable} to current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'inferior':
                    try:
                        z_val = float(value)
                        robot.MoveLinRelTrf(0, 0, -z_val, 0, 0, 0)
                        print(f'Move {z_val} mm {variable} to current')
                    except ValueError:
                        print(f"Invalid value: {value}")

                elif variable == 'downward':
                    try:
                        x_val = float(value)
                        robot.MoveLinRelTrf(x_val, 0, 0, 0, 0, 0)
                        print(f'Move {x_val} mm {variable} to current')
                    except ValueError:
                        print(f"Invalid value: {value}")
                
                elif variable == 'upward':
                    try:
                        x_val = float(value)
                        robot.MoveLinRelTrf(-x_val, 0, 0, 0, 0, 0)
                        print(f'Move {x_val} mm {variable} to current')
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
                break
            except Exception as e:
                print(f"Error: {e}")

    # Exiting the "with" block automatically disconnects from the robot.
    print('Now disconnected from the robot.')