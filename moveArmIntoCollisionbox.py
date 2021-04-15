#! /usr/bin/env python

# there is no launch file for this file

import rospy
import time
import tf
from SimpleNamespace import SimpleNamespace

# imports for Moveit
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def roboArmFollow(name):
    pass

def run():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_node')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)


    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    
    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""  
   

    rospy.loginfo("First point - before the box")
    
    start_pose = geometry_msgs.msg.Pose()
    start_pose.orientation.w = 1.0
    start_pose.position.x = 0.1
    start_pose.position.y = 0.0
    start_pose.position.z = 0.2

    group.set_pose_target(start_pose)
    
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()



    rospy.loginfo("Second point - into the box")
    
    start_pose = geometry_msgs.msg.Pose()
    start_pose.orientation.w = 1.0
    start_pose.position.x = 0.2
    start_pose.position.y = 0.0
    start_pose.position.z = 0.2

    group.set_pose_target(start_pose)
    
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()










    # listen for objects now
    listener = tf.TransformListener()



    rate = rospy.Rate(1)
    startTime = rospy.get_time()    
    
    group.set_planning_time(0.1) # seconds
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass