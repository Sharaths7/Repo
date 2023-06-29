#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import(
    String as str, 
    Bool, 
    Int8
) 
from ast import literal_eval
from utils import *
import parameters as param
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray
)
from farming_bot_msgs.msg import (
    ModeSwitchRequestMessage
)

from nav_msgs.msg import (
    Odometry
)

modes = [
    ModeSwitchRequestMessage.NAVIGATION,
    ModeSwitchRequestMessage.START_NAVIGATION,
    ModeSwitchRequestMessage.STOP_NAVIGATION,
    ModeSwitchRequestMessage.KEY_HOLE_GENERATION,
    ModeSwitchRequestMessage.ASSISTED_ALIGNMENT,
    ModeSwitchRequestMessage.START_HARVESTING,
    ModeSwitchRequestMessage.STOP_HARVESTING,
    ModeSwitchRequestMessage.DO_HARVESTING,
]



class Align(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['align','harvesting','navigate', 'decide'],
                            input_keys=['align','in_navigation','in_harvest','start_nav','row_side', 'current_plan'], 
                            output_keys=['align','out_navigation','out_harvest','start_nav','row_side','current_plan'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))
    def mode_feedback(self, msg):
            self.mode_feedback = msg.next_mode
    def execute(self, userdata):
        userdata.current_state = 'ASSISTED_ALIGNMENT'
        if self.mode_feedback == 'ASSISTED_ALIGNMENT':
            return 'align'
        else:
            if self.mode_feedback == 'START_NAVIGATION':
                self.mode_change('NAVIGATION')
                return 'navigate'
            if self.mode_feedback == 'START_HARVESTING':
                return 'harvesting'
        return 'align'
