#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from Heart_Beat.srv import *

Modules_Failed=[]
# Sleep Interval in secs 
# Used as waiting time for succesive check of heart beat
sleep_interval = 1
brake_name = 'Heart_Beat'
brake_status = False
def beats_service():
    rospy.wait_for_service('beats_service')
    try:
        beats = rospy.ServiceProxy('beats_service', Beats)
        resp1 = beats()
        return resp1.failed_modules
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def brake_status_callback(msg):
    print('asdasdadassdsadasd')
    global brake_status,brake_name
    brake_list = msg.data.split(",")
    # print(brake_list)    
    if(brake_name in brake_list):
        brake_status = True
        # print(3)
    else:
        brake_status= False 
        # print(2)


if __name__ == '__main__':
    rospy.init_node('Check_heart_Beat_Node')
    print('Check Heart Beat Node is Running')
    
    rate =rospy.Rate(2)
    rospy.Subscriber("/brake_status",String,brake_status_callback,queue_size=1)
    pub = rospy.Publisher("/guardian_topic",String,queue_size=4)

    while(1):
    
        failed = beats_service()
        
        length = 0
        if(failed!=None):
            length= len(failed)

        if(length==0):
            if(failed!=Modules_Failed):
                Modules_Failed=failed
                if(brake_status == True):
                    print("Heart Beat is Good !! ")
                    try:
                        # rospy.wait_for_service('emergencyStop')
                        emergencyStopSrv=rospy.ServiceProxy('emergencyStop',emergencyStop)
                        emergencyStopSrv(0,0,brake_name)
                    except rospy.ServiceException as e:
                        print("Service call failed: %s"%e)
                    # brake_status = False
        else:

            out = ""
            for i in failed:
                out+=i+","
            if(failed!=Modules_Failed):
                pub.publish(out)
                Modules_Failed=failed
                print("Heart Beat is checked and the failed modules are ")
                print(failed)
            # rospy.wait_for_service('stopCart')
            # stopCartSrv=rospy.ServiceProxy('stopCart',stopCart)
            # stopCartSrv(1)
            # time.sleep(1)
            if(brake_status == False):
                try:
                    # rospy.wait_for_service('emergencyStop')
                    emergencyStopSrv=rospy.ServiceProxy('emergencyStop',emergencyStop)
                    emergencyStopSrv(1,0,brake_name)
                except rospy.ServiceException as e:
                        print("Service call failed: %s"%e)
        time.sleep(0.25)             

    # rospy.sleep(sleep_interval)
    
    rospy.spin()