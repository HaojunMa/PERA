#!/usr/bin/env python

"""
    trajectory_demo.py - Version 0.1 2014-01-14
    
    Send a trajectory to the FollowJointTrajectoryAction server
    Adapted for the pera
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving gripper
        sync = rospy.get_param('~sync', True)
        
        # Which joints define the arm?
        arm_joints = ['shoulder_yaw_joint',
                      'shoulder_pitch_joint',
                      'elbow_pitch_joint', 
                      'elbow_roll_joint',
                      'shoulder_roll_joint',
                      'wrist_pitch_joint',
                      'wrist_yaw_joint' ]
        
        # Which joints define the gripper?
        gripper_joints = [ 'gripper_joint']
        

        if reset:
            # Set the arm back to the resting position
            arm_goal  = [0, 0, 0, 0, 0, 0, 0]
            
            # Re-center the gripper
            gripper_goal = [0.2]  
        else:
            # Set a goal configuration for the arm
            arm_goal  = [-1.2, 1, 0.5, 0,2, -0.5, 0, 0]
            
            # Set a goal configuration for the gripper
            gripper_goal = [0.8]
    
        # Connect to the arm trajectory action server
        rospy.loginfo('Waiting for arm trajectory controller...')
        
        arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        
        # Connect to the gripper trajectory action server
        rospy.loginfo('Waiting for gripper trajectory controller...')
    
        gripper_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       
        gripper_client.wait_for_server()
        
        rospy.loginfo('...connected.')    
    
        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
        
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal to the action server
        arm_client.send_goal(arm_goal)
        
        if not sync:
            # Wait for up to 5 seconds for the motion to complete 
            arm_client.wait_for_result(rospy.Duration(5.0))
        
        # Create a single-point gripper trajectory with the gripper_goal as the end-point
        gripper_trajectory = JointTrajectory()
        gripper_trajectory.joint_names = gripper_joints
        gripper_trajectory.points.append(JointTrajectoryPoint())
        gripper_trajectory.points[0].positions = gripper_goal
        gripper_trajectory.points[0].velocities = [0.0 for i in gripper_joints]
        gripper_trajectory.points[0].accelerations = [0.0 for i in gripper_joints]
        gripper_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        print 'Gripper goal', gripper_goal

        # Send the trajectory to the gripper action server
        rospy.loginfo('Moving the gripper to goal position...')
        
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory = gripper_trajectory
        gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal
        gripper_client.send_goal(gripper_goal)
        

        # Wait for up to 5 seconds for the motion to complete 
        gripper_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
