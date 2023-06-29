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

class Harvest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','harvesting'],
                            input_keys=['in_navigation','in_harvest','start_nav','row_side','current_plan',  'current_state','plan_execution_status'], 
                            output_keys=['out_navigation','in_harvest','out_harvest','start_nav','row_side','current_plan',  'current_state', 'plan_execution_status'])
        # self.align_successdef execute(self, userdata):
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
        self.start_harvest = False
        self.harvest_execute = None 
        self.current_location = [0,0,0]
        self.track_odometry_cartsian =rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.publish = True
        self.goal_to_finish_harvest = None

    def callback_start(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def mode_feedback(self, msg):
        self.mode_feedback = msg.next_mode
    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))

    def execute(self, userdata):
        userdata.current_state = 'HARVEST'
        while self.mode_feedback != 'ASSISTED_ALIGNMENT':
            self.harvest_execute = userdata.current_plan
            # rospy.loginfo(" Plan: Harvest till : ", self.harvest_execute[0])
            self.start_harvest = True
            self.goal_to_finish_harvest = convert_goal_pose(self.harvest_execute[1])
            while self.mode_feedback!='False':
                self.distance_to_goal = euclidean_distance(self.current_location,self.harvest_execute[1])
                print(self.distance_to_goal)
                if self.distance_to_goal < param.threshold_distance_to_goal:
                    self.mode_change('STOP_HARVESTING')
                    self.start_harvest = False
                    publish = True
                    return 'done'
                return 'harvesting'

        # userdata.plan[0:0] = userdata.current_plan
        self.mode_change('ASSISTED_ALIGNMENT')
        if self.mode_feedback == 'ASSISTED_ALIGNMENT':
            return 'align'