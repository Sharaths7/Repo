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


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['decide','running','align'],
                            input_keys=['in_navigation','in_harvest','start_nav','row_side','current_plan','previous_state ',  'current_state', 'plan_execution_status'], 
                            output_keys=['out_navigation','in_harvest','out_harvest','start_nav','row_side','current_plan','previous_state ',  'current_state','plan_execution_status'])
        # self.align_success
        self.publish = True
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.row_side_data = None
        self.plan = [['Navigate',[15,15,0]],['Harvest',[]],['Navigate',[25,15,0]],['Harvest',[]]]
        self.goal = [10,10,0]
        self.pub_goal = rospy.Publisher("/gplanner/goal", PoseStamped, queue_size=1)
        # self.plan_data =rospy.Subscriber("/plan_to_execute", Int64MultiArray, self.receive_plan)
        self.track_odometry_cartsian =rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
        self.current_location = [0,0,0]
        self.distance_to_goal = None
        self.nav_status = None
        self.nav_sequence = None
        self.nav_sequence_sub = rospy.Subscriber("/gplanner/navigation_sequence", Int8, self.track_navigation_sequence)
        self.nav_status_sub = rospy.Subscriber("/gplanner/navigation_status", str, self.track_navigation_status)

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
    def track_navigation_status(self, msg):

        self.nav_status = msg.data
        return self.nav_status
    
    def track_navigation_sequence(self, msg):
        self.nav_sequence = msg.data
        return self.nav_sequence



    def execute(self, userdata):
        userdata.current_state = 'NAVIGATION'
        while self.mode_feedback != 'ASSISTED_ALIGNMENT':
            # rospy.loginfo(" Plan: Navigate to : ", plan[0])
            # print(plan,plan[1])
            self.start_nav = True
            userdata.plan_execution_status = False
            goal_to_achieve = convert_goal_pose(userdata.current_plan[1])
            # rospy.sleep(1) 
            publish = True
            while self.start_nav!=False and self.mode_feedback != 'ASSISTED_ALIGNMENT':
                self.distance_to_goal = euclidean_distance(self.current_location,userdata.current_plan[1])
                if publish == True:
                    print('Publishing Goal and state')
                    self.pub_goal.publish(goal_to_achieve)
                    self.mode_change('START_NAVIGATION')
                    publish = False


                # self.start_nav = False
                # return 'try'
                if self.distance_to_goal < param.threshold_distance_to_goal:
                    self.mode_change('STOP_NAVIGATION')
                    self.start_nav = False
                    userdata.plan_execution_status = True
                    publish = True
                    return 'decide'
                return 'running'

        userdata.plan[0:0] = userdata.current_plan
        self.mode_change('ASSISTED_ALIGNMENT')
        if self.mode_feedback == 'ASSISTED_ALIGNMENT':
            return 'align'
    