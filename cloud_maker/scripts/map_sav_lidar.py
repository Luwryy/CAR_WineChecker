#!/usr/bin/env python3

import sys
import os
import shutil
import copy
import rospy
import subprocess
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospkg

#Default path for rtabmap's database
db_path = "~/.ros/rtabmap.db"
sav_path = "~/catkin_mix/bags"

db_length = len("db_path:=")
sav_length = len("sav_path:=")

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    rospy.loginfo(list_output)
    string_list = str(list_output)[2:-1]
    rospy.loginfo("End of find separator routine")
    for word in string_list.split("\\n"):
        
        if (word.startswith(s)):
            rospy.loginfo("Now killing"+word)
            os.system("rosnode kill " + word)
    os.system("rosnode kill " + "/map_saving/ply_write")

def startRecord(sav_dir):
    store_folder = os.path.expanduser(sav_dir)
    #rospy.logwarn(store_folder)

    new_folder = "fpoint_cloud_1";i = 1
    while(os.path.exists(store_folder+'/'+new_folder) and i < 1000):
        i+=1
        new_folder = "fpoint_cloud_%s"% i
    else:
        if(i < 1000):
            new_folder = "fpoint_cloud_%s"% i

            path = "%s/%s"%(store_folder,new_folder)
            rospy.logwarn(path)
            os.mkdir(path)

            command = "roslaunch cloud_maker save_to_ply.launch folderpath:=%s "%path
            rospy.logwarn(command)
            ply_saver = subprocess.Popen(command, shell=True)

            return ply_saver

        else:
            rospy.logwarn("Could not find a name for the bag, please make some room in"+store_folder)
            return 1

def record(data, args):
    global ply_saver #= args[0]
    sav_dir = args

    #If the given command is good
    if data == String("save"): 

        #Start a new one
        ply_saver = startRecord(sav_dir)
        
        rospy.sleep(2)

        terminate_ros_node("/map_saving/ply_write")
        
        try:
            if(ply_saver == 1):
                return 1
        except:
            rospy.loginfo("Map saving done")
            
        #Return the bag's handle to allow to close it at the next call back
        
    else:
        rospy.logwarn("Input not recognized, please type 'save' if you wish to save the current cloud map")
        return 1

def map_recorder( sav_dir):
    global ply_saver
    rospy.init_node('mapRecord', anonymous=True)
    rate = rospy.Rate(2) # 2hz

    rospy.Subscriber('/map_record', String, record,( sav_dir))

    try:
        if(ply_saver == 1):
            rospy.logerr("Map saver shutting down")
            return 1
    except:
        rospy.loginfo("Map saver rearmed")

    rospy.spin()

    

if __name__ == '__main__':
    sav_dir = os.path.expanduser(sav_path)
    ply_saver = 0

    if len(sys.argv) != 4 and len(sys.argv) != 5:
        rospy.logwarn("Map_recorder: Data base or Save path are not specified")

        map_recorder( sav_dir)

    else:
        #Keep the same structure as map_sav.py
        for i in range(len(sys.argv)):
            db_index = (sys.argv[i]).find("db_path:=")
            sav_index = (sys.argv[i]).find("sav_path:=")
            if(db_index+1):
                #Allow to get a usable path be the os lib if the provided data base path begins with '~'
                data_base = os.path.expanduser((sys.argv[i])[(db_index+db_length):])
                
            elif(sav_index+1):
                sav_dir = os.path.expanduser((sys.argv[i])[(sav_index+sav_length):])
        
        
        map_recorder( sav_dir)
        
