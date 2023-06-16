#!/usr/bin/env python3

import sys
import os
import shutil
import copy
import rospy
import roslaunch
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospkg
#
#sensor_msgs/PointCloud2

#Default path for rtabmap's database
db_path = "~/.ros/rtabmap.db"
sav_path = "~/catkin_mix/databases"

db_length = len("db_path:=")
sav_length = len("sav_path:=")


def rtab_resume():
    rospy.wait_for_service('/rtabmap/resume')
    try:
        func1 = rospy.ServiceProxy('/rtabmap/resume', Empty)
        func1()
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return 1

    rospy.wait_for_service('/rtabmap/resume_odom')
    try:
        func2 = rospy.ServiceProxy('/rtabmap/resume_odom', Empty)
        func2()
        return 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return 1

def delete_db(data_base):
    if os.path.exists(data_base):
            os.remove(data_base)
            rospy.loginfo("Previous data base deleted") 
    else:
        rospy.logwarn("The database does not exist and couldn't be deleted")     

def rtab_pause():
    rospy.wait_for_service('/rtabmap/pause')
    try:
        func = rospy.ServiceProxy('/rtabmap/pause', Empty)
        func()
        return 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return 1

def dataString(data):
    return data

def record(data, args):
    
    db_path = args[0]
    sav_dir = args[1]

    #If the given command is good
    if data == String("save"): 


        rtab_pause()

        db_store = os.path.expanduser(sav_dir)

        db_name = "rtabmap1.db";i = 1
        while(os.path.exists(db_store+'/'+db_name) and i < 1000):
            i+=1
            db_name = "rtabmap%s.db"% i
        else:
            if(i < 1000):
                db_name = "rtabmap%s.db"% i
            else:
                rospy.logwarn("Could not find a name for the data base, please make some room in"+db_store)
                return 1

        original = r'%s'% db_path
        target = r'%s/%s' % (db_store, db_name)

        shutil.copyfile(original, target)

        rtab_resume()

        rospy.loginfo("Database saved, mapping will resume")

        return 0
    else:
        rospy.logwarn("Input not recognized, please type 'save' if you wish to save the current cloud map")
        return 1

def map_recorder(db_path, sav_dir):

    rospy.init_node('mapRecord', anonymous=True)
    rate = rospy.Rate(2) # 2hz

    rospy.Subscriber('/map_record', String, record,(db_path, sav_dir))

    rospy.spin()

    

if __name__ == '__main__':
    data_base = os.path.expanduser(db_path)
    sav_dir = os.path.expanduser(sav_path)

    if len(sys.argv) != 4 and len(sys.argv) != 5:
        rospy.logwarn("Map_recorder: Data base or Save path are not specified")
        
        delete_db(data_base)
        map_recorder(data_base, sav_dir)

    else:
        for i in range(len(sys.argv)):
            db_index = (sys.argv[i]).find("db_path:=")
            sav_index = (sys.argv[i]).find("sav_path:=")
            if(db_index+1):
                #Allow to get a usable path be the os lib if the provided data base path begins with '~'
                data_base = os.path.expanduser((sys.argv[i])[(db_index+db_length):])
                
            elif(sav_index+1):
                sav_dir = os.path.expanduser((sys.argv[i])[(sav_index+sav_length):])

        delete_db(data_base)
        map_recorder(data_base, sav_dir)
        

