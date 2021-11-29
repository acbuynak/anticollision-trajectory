#!/usr/bin/env python

# TODO: add license

# A simple example showing the use of an Action client to request execution of
# a complete JointTrajectory.
#
# Note: joint names must match those used in the urdf, or the driver will
#       refuse to accept the goal.


import rospy
import actionlib
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class SimpleTrajectoryActionClient():

    def __init__(self, joint_names):
        """
        Initialize ROS Node.
        Setup action client.
        Setup trajectory plan object.

        param: joint_names. Example: ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
        """

        # Setup Node
        rospy.init_node('simple_trajectory_action_client', log_level=rospy.DEBUG)

        # Create client and make sure it's available
        self.client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for driver\'s action server to become available ..')
        self.client.wait_for_server()
        rospy.loginfo('Connected to trajectory action server')

        # Setup simple goal
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()

        # Set Joint Names
        self.goal.trajectory.joint_names = joint_names

        # motoman_driver only accepts goals with trajectories that start at the
        # current robot state, so retrieve that and use it as the first point in
        # the trajectory
        self.robot_joint_states = rospy.wait_for_message('joint_states', JointState)

        # make sure the state we get contains the same joints (both amount and names)
        if set(self.goal.trajectory.joint_names) != set(self.robot_joint_states.name):
            rospy.logfatal("Mismatch between joints specified and seen in current "
                "JointState. Expected: '{}', got: '{}'. Cannot continue.".format(
                    ', '.join(self.robot_joint_states.name),
                    ', '.join(self.goal.trajectory.joint_names)))
            sys.exit(1)

        # Set First Waypoint as Current Position (w/ velocity 0)
        self.goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions       = self.robot_joint_states.position,
                velocities      = [0.0] * len(self.goal.trajectory.joint_names),
                time_from_start = rospy.Duration( 0.0)
                )
            )
        print([0.0] * len(self.goal.trajectory.joint_names))


    def add_joint_waypoint(self, joint_values, time_posn, vel):
        """
        Add waypoint to trajectory plan.

        param:  joint_values:   ordered list of joint values
        param:  time_posn:      time (sec) when waypoint should be achieved
        param:  vel:            velocity of trajectory when waypoint achieved
        """

        newpoint = JointTrajectoryPoint(
            positions       = joint_values,
            velocities      = vel, 
            time_from_start = rospy.Duration(time_posn)
            )
        self.goal.trajectory.points.append(newpoint)


    def send_trajectory(self):
        """
        Goal constructed, submit it for execution
        """
        rospy.loginfo("Submitting goal ..")
        self.client.send_goal(self.goal)
        rospy.loginfo("Waiting for completion ..")
        self.client.wait_for_result()
        rospy.loginfo('Done.')



