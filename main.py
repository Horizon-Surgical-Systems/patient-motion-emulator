import mecademicpy.robot as mdr
import numpy as np
import os
import time
import argparse
import Parameter
from TrajectoryPlanner import TrajectoryPlanner
from TrajectoryUtils import TrajectoryUtils

if __name__ == "__main__":

    # Create the mutually exclusive group
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--x", "-x", action='store_true', help="Enable X trajectory")
    group.add_argument("--y", "-y", action='store_true', help="Enable Y trajectory")
    group.add_argument("--z", "-z", action='store_true', help="Enable Z trajectory")
    group.add_argument("--ux", "-ux", action='store_true', help="Enable UX trajectory")
    group.add_argument("--uy", "-uy", action='store_true', help="Enable UY trajectory")
    group.add_argument("--uz", "-uz", action='store_true', help="Enable UZ trajectory")
    group.add_argument("--linear", "-l", action='store_true', help="Enable arbitrary linear trajectory")
    args = parser.parse_args()

    robot = mdr.Robot()
    
    # Initialize trajectory planner
    trajectory_planner = TrajectoryPlanner(
        start_point=np.array([0.0, 0.0, 0.0]),
        end_point=np.array([0.5, 0.5, 0.5]),
        max_velocity=Parameter.MAX_VELOCITY,
        max_acceleration=Parameter.MAX_ACCELERATION
    )

    # Generate trajectory
    if args.x:
        trajectory = trajectory_planner.calculate_x_trajectory()
    elif args.y:
        trajectory = trajectory_planner.calculate_y_trajectory()
    elif args.z:
        trajectory = trajectory_planner.calculate_z_trajectory()
    elif args.ux:
        trajectory = trajectory_planner.calculate_ux_trajectory()
    elif args.uy:
        trajectory = trajectory_planner.calculate_uy_trajectory()
    elif args.uz:
        trajectory = trajectory_planner.calculate_uz_trajectory()
    elif args.linear:
        trajectory = trajectory_planner.calculate_linear_trajectory()

        
    # Apply trajectory smoothing and padding
    trajectory = TrajectoryUtils.trajectory_smoothing(trajectory)
    trajectory = TrajectoryUtils.trajectory_padding(trajectory)

    # robot.Connect(address=Parameter.ROBOT_IP_ADDRESS)