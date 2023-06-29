#!/usr/bin/env python

import sys
import rospy
from subprocess import call
from Heart_Beat.srv import CheckReqs,CheckReqsResponse

# Sleep Interval in secs 
# Used as waiting time for succesive check whether the launching module 
# met the requirements
sleep_interval = 5

#The command line argument received to launch that particular module
try:
    launch_module = sys.argv[1]
except:
    print("No module name passed")
    sys.exit(0)

# Utilities to run bash script from python
cmd = ['gnome-terminal','-e']
cmd_tab = ['gnome-terminal','--tab','-e']

launch_modules = {
            'NDT':"'echo NDT;$SHELL'"
            ,'3D_LIDAR':"'echo 3D_LIDAR;$SHELL'"
            ,'2D_LIDAR':"'echo 2D_LIDAR;$SHELL'"
        }

if(launch_module not in launch_modules):
    print("There is no module named "+launch_module)

def check_req(launch_module):
    rospy.wait_for_service('check_reqs')
    try:
        check_reqs = rospy.ServiceProxy('check_reqs', CheckReqs)
        resp1 = check_reqs(launch_module)
        return resp1.out
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


count = 5
while(count>0):
    if(check_req(launch_module)):
        curr = cmd[:]
        curr_cmd = "bash -c " + launch_modules[launch_module]
        curr.append(curr_cmd)
        #opens a new terminal and  run the bash command
        call(curr)
        break
    else:
        count-=1
    rospy.sleep(sleep_interval)

sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('Launch_File_Node')
    rate =rospy.Rate(2)
    rospy.spin()