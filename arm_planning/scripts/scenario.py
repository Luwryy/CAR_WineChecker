#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg 
import geometry_msgs.msg
from geometry_msgs.msg import Twist
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

def arm_move(Pose):
	global group
	group.set_named_target(Pose)
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot_description = rospy.get_param("/robot_description")
#rospy.sleep(10)
robot = moveit_commander.RobotCommander(robot_description= "/robot_description")
#scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator",robot_description= "/robot_description")#, wait_for_servers = 5)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size = 10)
speed_publisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size = 10)

pose_target = geometry_msgs.msg.Pose()

speed = [Twist(), Twist()]

speed[0].linear = geometry_msgs.msg.Vector3(0,0,0)
speed[0].angular = geometry_msgs.msg.Vector3(0,0,0)
speed[1].linear = geometry_msgs.msg.Vector3(0.7,0,0)
speed[1].angular = geometry_msgs.msg.Vector3(0,0,0)


#Pose-driven (=joint-driven)
if len(sys.argv) != 2:
    rospy.loginfo("No posessssssssssss specified")
    group.set_named_target("Middle_center")
else:
    try:
        group.set_named_target(sys.argv[1])
    except:
        rospy.logerr("Unknown pose, try entering an existing pose")
group.set_goal_tolerance(0.02)

#Scenario
arm_move("Close_center")
rospy.sleep(10)

speed_publisher.publish(speed[1])
rospy.sleep(7)
speed_publisher.publish(speed[0])

arm_move("Middle_right3")
rospy.sleep(1)
arm_move("High_right3")
rospy.sleep(1)
arm_move("Close_right3")
arm_move("Close_left3")
arm_move("High_left3")
rospy.sleep(1)
arm_move("Middle_left3")
rospy.sleep(1)
arm_move("Close_center")

speed_publisher.publish(speed[1])
rospy.sleep(7)
speed_publisher.publish(speed[0])

arm_move("Middle_right3")
rospy.sleep(1)
arm_move("High_right3")
rospy.sleep(1)
arm_move("Close_right3")
arm_move("Close_left3")
arm_move("High_left3")
rospy.sleep(1)
arm_move("Middle_left3")
rospy.sleep(1)
arm_move("Close_center")

#rospy.sleep(10)


moveit_commander.roscpp_shutdown()
#rospy.spin()
