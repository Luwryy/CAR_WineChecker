#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg 
import geometry_msgs.msg
from numpy import cos
from numpy import sin


#roll (x), pitch (Y), yaw (z)
def eulerToQuaternion(roll, pitch, yaw): 

    # Abbreviations for the various angular functions

    cr = cos(float(roll) * 0.5)
    sr = sin(float(roll) * 0.5)
    cp = cos(float(pitch) * 0.5)
    sp = sin(float(pitch) * 0.5)
    cy = cos(float(yaw) * 0.5)
    sy = sin(float(yaw) * 0.5)

    q = [0,0,0,0]
    q[0] = cr * cp * cy + sr * sp * sy
    q[1] = sr * cp * cy - cr * sp * sy
    q[2] = cr * sp * cy + sr * cp * sy
    q[3] = cr * cp * sy - sr * sp * cy

    return q





moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot_description = rospy.get_param("/robot_description")
#rospy.sleep(10)
robot = moveit_commander.RobotCommander(robot_description= "/robot_description")
#scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator",robot_description= "/robot_description")#, wait_for_servers = 5)

#rospy.sleep(30)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 10)

pose_target = geometry_msgs.msg.Pose()

#Position-driven
"""
if len(sys.argv) >= 4:
    pose_target.position.x = float(sys.argv[1])
    pose_target.position.y = float(sys.argv[2])
    pose_target.position.z = float(sys.argv[3])
else:
    rospy.loginfo("No position specified")
    pose_target.position.x = 0.3
    pose_target.position.y = 0.1
    pose_target.position.z = 0.3



if len(sys.argv) == 7:
    q = eulerToQuaternion(sys.argv[4], sys.argv[5], sys.argv[6])
else:
    rospy.loginfo("No angle specified")
    q = eulerToQuaternion(1,1,1)

pose_target.orientation.w = q[0]
pose_target.orientation.x = q[1]
pose_target.orientation.y = q[2]
pose_target.orientation.z = q[3]

rospy.loginfo("Planned position:")
rospy.loginfo(pose_target)

actual_pose = group.get_current_pose()
rospy.loginfo("Current position:")
rospy.loginfo(actual_pose)"""

#Pose-driven (=joint-driven)
if len(sys.argv) != 2:
    rospy.loginfo("No pose specified")
    group.set_named_target("Middle_center")
else:
    try:
        group.set_named_target(sys.argv[1])
    except:
        rospy.logerror("Unknown pose, try entering an existing pose")


#group.set_pose_target(pose_target)

group.set_goal_tolerance(0.02)

success = group.go(wait=True)
group.stop()
group.clear_pose_targets()
#plan1 = group.plan()
#rospy.sleep(10)
#group.go(wait=True)


moveit_commander.roscpp_shutdown()
#rospy.spin()