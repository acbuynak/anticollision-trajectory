#!/usr/bin/env python

# TODO: add license

# A simple example showing the use of the trajectory_action_client.py tool.
#
# Note: joint names must match those used in the urdf, or the driver will
#       refuse to accept the goal.


## Imports
from trajectory_action_client import SimpleTrajectoryActionClient
import numpy as np
import csv
import os

import rospy
import actionlib
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


## Main Function
def main():
    """
    Main Function. 
    1 - Setup trajectory action client
    2 - Build trajectory
        Use pre-planned poses from an CSV file
        Set timing for when to achieve each pose
        Set Velocity and Acceleration desired at each pose waypoint
    3 - Send trajectory to robot for execution
    """

    # Parameters
    joint_names = rospy.get_param('/controller_joint_names')
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)

    # Load Trajectory Plan (list of joints)
    cwd = os.path.dirname(os.path.realpath(__file__))
    traj_plan_file_name = "trajectory.csv"
    traj_plan_file_path = open(os.path.join(cwd,traj_plan_file_name))

    imported_array = np.genfromtxt(traj_plan_file_path, delimiter="|")
    traj_array = imported_array[1:,:]
    traj_array[:,0] = traj_array[:,0][:-4]
    rospy.loginfo("Imported Array of Shape: " + str(np.shape(traj_array)))

    print(traj_array[0:5,0])

    # Loop through Trajectory Plan
    # Note: velocities should be all 0 for the first and last trajectory point
    for i, row in enumerate(traj_array):
        rowx = np.ndarray.tolist(row)
        # rowx = [0 if x != x else x for x in rowx]


        tt = float(rowx[0])
        posn = rowx[1:7]
        velo = rowx[7:13]
        accl = rowx[13:19]

        traj_plan.add_joint_waypoint(tt, posn, velo, accl)
        rospy.logdebug("Added Waypoint #%s.  Waypoint: %s", i, rowx)
    
    # Send Plan
    traj_plan.send_trajectory()


if __name__ == '__main__':
    main()