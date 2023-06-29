#!/usr/bin/env python

import rospy
import rostopic
from std_msgs.msg import String,Float32, Float64,Int64,Int16,UInt16
from base_global_planner_segment.srv import CheckReqs,CheckReqsResponse, Beats, BeatsResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Path
import os
import logging
import sys
from datetime import datetime
from nav_msgs.msg import (
    Odometry
)


dir_add= '/home/infy/Heart_Beat_WS/log'
log_dir = os.listdir(dir_add)
full_path = ["{1}/{0}".format(x,dir_add) for x in log_dir]

if len(log_dir) == 3:
    oldest_file = min(full_path, key=os.path.getctime)
    os.remove(oldest_file)

def logger_setup(name):
    
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    handler = logging.FileHandler(dir_add+'/{:%Y-%m-%d}.log'.format(datetime.now()), mode='a')
    handler.setFormatter(formatter)
    screen_handler = logging.StreamHandler(stream=sys.stdout)
    screen_handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.addHandler(screen_handler)
    return logger

logger = logger_setup('Harvey_Heart_Beat_v1')


# upper threshold value until which the topic is assumed to be working fine (in secs)
# beat_timer = 10

# lower threshold value of the frequency until which the 
# topic is assumed to be working fine (in hz)
# lower_hz = 1

# no of past time stamps to save for calculating frequency
no_of_timer = 20
accepted_message_class = [Float64,Float32,String,Int64,Int16,UInt16]

def common_callback(status,topic):
    # print('Inside Callback for the topic  ' + topic)
    # print(status,topic)
    global modules
    modules[topic][3]=modules[topic][3][1:no_of_timer]+ [rospy.Time.now().secs]
    if(modules[topic][5]!=-1):
        if(status!=""):
            modules[topic][0]=1
        else:
            modules[topic][0]=0
    if(modules[topic][6] in accepted_message_class):
        
        modules[topic][8] = str(status.data)
    # print(modules[topic])
    # check_beats()

def check_beats(req):
    global logger
    out = []
    logging_data= "" 
    for i in modules:
        if(modules[i][5] != -1):
            if(modules[i][0]==-1 ):
                logging_data += "|  "+i+" Module is not set |"
                continue
            if(modules[i][0]==0):
                print("Module " + i + " Has No Data Being Published ")
                logging_data +="| Module " + i + " Has No Data Being Published |"
                continue
            time = modules[i][3]
            if(rospy.Time.now().secs - time[-1] > modules[i][4]):
                print("Module " + i + " Failed as there is no data for " + str(modules[i][4]) + " secs ")
                logging_data +=" | Module " + i + " Failed as there is no data for " + str(modules[i][4]) + " secs |"
                
                out.append(i)
            else:
                time = [j for j in time if j]
                diff = max(time)-min(time)
                if(diff == 0):
                    diff = 1
                hz = float(len(time))/diff
                # print(time)
                print("calculated hz for Node " + i +" is " + str(hz))
                logging_data +="| Module " + i + "  hz is " + str(hz)+"  |"
                if(len(time)<no_of_timer):
                    
                    continue
                if(hz<modules[i][5]):
                    print("Module " + i + " Failed as the frequency in hz is less than " + str(modules[i][5]))
                    logging_data +="| Module " + i + " Failed as the frequency in hz is less than " + str(modules[i][5])+"  |"
                    
                    # modules[i][3]= [None]*no_of_timer
                    out.append(i)
            if(modules[i][8]!=[]):
                logging_data += " data: "+str(modules[i][8])
        else:
            logging_data +="| Module " + i + str(modules[i][8])
        
                
    # print(out)
    # print(logging_data)
    logger.info(logging_data)
    resp = BeatsResponse()
    resp.failed_modules = out
    return resp

function = common_callback


# Module_name : 
# [ status, topic name, callback_function_name, timestamps[no_of_timer], no of secs to report after topics stoped publishing, 
# lower frequency of the topic in hz till which the topic is working fine,
# topic type, [dependencies list],data from topic ]

modules = {
            'Odometry':[-1,'/odometry/filtered_map',function,[None]*no_of_timer,1,8,Odometry,[],[]]
           

            # ,'VOICE_ASSISTANT':[-1,'/tbd',function,[None]*no_of_timer,1,5,String,['NDT']]
            # ,'RP_Right':[-1,'/rp1/scan',function,[None]*no_of_timer,2,2,Float32,[]]

        }

# 'BEHAVIOUR_PLANNER','LOCAL_PLANNER','MPC','DBW','EMERGENCY_BRAKE'

def check_requirements(launch_method):
    for i in modules[launch_method][7]:
        if(modules[i][0]!=1):
            return False
    return True

def handle_check_reqs(req):
    # print(req.module)
    o = check_requirements(req.module)
    resp = CheckReqsResponse()
    resp.out = o
    return resp


if __name__ == '__main__':
    rospy.init_node('HEART_BEAT')
    for i in modules:
        rospy.Subscriber(modules[i][1],modules[i][6], modules[i][2],i,queue_size=1)
        print('     #############   Subscribing to module ',i)

    s = rospy.Service('check_reqs', CheckReqs, handle_check_reqs)
    b = rospy.Service('beats_service',Beats, check_beats)
    rate =rospy.Rate(2)
    rospy.spin()