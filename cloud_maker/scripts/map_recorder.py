#!/usr/bin/env python3

import sys
import os
import copy
import rospy
import roslaunch
from std_msgs.msg import String
#from sensor_msgs.msg import PointCloud2
import rospkg

#
#sensor_msgs/PointCloud2


"""
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = [package_path+'/launch/pointcloud_to_pcd.launch', 'input:=/rtabmap/cloud_map']

roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
roslaunch_args = cli_args[1:]

roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

save_to_pcd = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

save_to_pcd.start()
print process.is_alive()
save_to_pcd.stop()

cloudMap = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/save_to_pcd.launch"])
cloudMap.start()
rospy.loginfo("started")
cloudMap.shutdown()

cloudBag = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/cloudBag.launch"])
cloudBag.start()
rospy.loginfo("started")
cloudBag.shutdown()

"""


"""
def rtab_resume():
    rospy.wait_for_service('/rtabmap/resume')
    try:
        func1 = rospy.ServiceProxy('/rtabmap/resume', std_srvs/Empty)
        resp = func1()
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.wait_for_service('/rtabmap/resume_odom')
    try:
        func2 = rospy.ServiceProxy('/rtabmap/resume_odom', std_srvs/Empty)
        resp += func2()
        return resp.truth
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)"""



"""
def publish_map():
    rospy.wait_for_service('/rtabmap/publish_map')
    try:
        #map = rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, dataString)
        func = rospy.ServiceProxy('/rtabmap/publish_map', rtabmap_msgs/PublishMap),
        
        resp, map = func(1, 1, 1), rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, dataString)
        return map
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return 1"""
"""
def rtab_pause():
    rospy.wait_for_service('/rtabmap/pause')
    try:
        func = rospy.ServiceProxy('/rtabmap/pause', std_srvs/Empty)
        resp = func()
        return resp.truth
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)"""

def dataString(data):
    return data

def record(data, args):
    cloudBag = args[0]
    cloudMap = args[1]

    #If the given command is good
    if data == String("save"): 

        #Closing the bag and saving the cloudmap
        cloudBag.shutdown()
        rospy.loginfo("Rosbag closed")
        cloudMap.start()
        rospy.sleep(1)
        cloudMap.shutdown()

        #Deleting the bag
        bag_path = os.path.expanduser("~/catkin_mix/bags/cloudBag1.bag")
        if os.path.exists(bag_path):
            #os.remove(bag_path)
        #else:
            rospy.logwarn("Bag couldn't be deleted")

        rospy.loginfo("The cloudmap is saved, please restart map_recorder from freenect_launch to save it again")

        return 0
    else:
        rospy.logwarn("Input not recognized, please type 'save' if you wish to save the current cloud map")
        return 1

def map_recorder():
    rp = rospkg.RosPack()
    package_path = rp.get_path('freenect_launch')
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cloudBag = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/cloudBag.launch"])
    cloudMap = roslaunch.parent.ROSLaunchParent(uuid, [package_path+"/launch/save_to_pcd.launch"])
    cloudBag.start()
    rospy.loginfo("Rosbag started")

    rospy.init_node('mapRecord', anonymous=True)
    rate = rospy.Rate(2) # 2hz

    rospy.Subscriber('/map_record', String, record,(cloudBag, cloudMap))

    rospy.spin()

    

if __name__ == '__main__':
    if len(sys.argv) != 4:
        rospy.logwarn("Map_recorder: Data base path not specified, the current sequence will append the last one made")
        
        map_recorder()
    else:
        #Allow to get a usable path be the os lib if the provided data base path begins with '~'
        data_base = os.path.expanduser(sys.argv[1])

        if os.path.exists(data_base):
            os.remove(data_base)
            rospy.loginfo("Previous data base deleted") 
            map_recorder()
        else:
            rospy.logwarn("The database does not exist and couldn't be deleted") 
            map_recorder()

