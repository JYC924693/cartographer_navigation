#!/usr/bin/env python

import rospy
import actionlib
import tf
from math import pi

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


waypoints = [  # This is our list of waypoints
    [(0.4, 0.4,0.0), tf.transformations.quaternion_from_euler(0, 0, 45*pi/180)],    # kiri dalam
    [(2.5, 0.4,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # kiri atas luar
    [(0.4, 0.4,0.0), tf.transformations.quaternion_from_euler(0, 0, 45*pi/180)],    # kiri dalam
    [(0.4, 2.5,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # kiri kiri luar
    
    [(0.4,-0.4, 0.0), tf.transformations.quaternion_from_euler(0, 0, 325*pi/180)],  # kanan dalam
    [(2.5,-0.4,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # kanan atas luar
    [(0.4,-0.4,0.0), tf.transformations.quaternion_from_euler(0, 0, 45*pi/180)],    # kanan dalam
    [(0.4,-2.5,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # kanan kanan luar    
]


def goal_pose(pose):  # Convert from waypoints into goal positions
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('maze_patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # Creates the Action Client
    client.wait_for_server()
    
    for pose in waypoints:   # Loop through each waypoint, sending them as goals
        goal = goal_pose(pose)
        client.send_goal(goal)
        print("Moving towards goal...")
        # wait for robot to move, but if more than 60 sec, send next goal/waypoint
        client.wait_for_result(rospy.Duration.from_sec(60.0))
        
    print("Completed.")
