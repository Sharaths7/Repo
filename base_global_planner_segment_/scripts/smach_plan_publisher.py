#!/usr/bin/env python

import rospy
from rospy_message_converter import message_converter
from std_msgs.msg import String
from ast import literal_eval


def talker():
    plan = [['Navigate',[10,10,0]],['Harvest',[15,10,0]],['Navigate',[15,15,180]],['Harvest',[20,15,0]]]
    message = str(plan)
    # message = 
    # mes = literal_eval(message)
    # message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', plan)
    # print(message)
    pub = rospy.Publisher('/plan', String, queue_size=10)
    rospy.init_node('plan_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(message)
    rospy.loginfo(type(message))
    pub.publish(message)

    # while not rospy.is_shutdown():
    #     rospy.loginfo(message)
    #     pub.publish(message)
    #     rate.sleep()
      
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



