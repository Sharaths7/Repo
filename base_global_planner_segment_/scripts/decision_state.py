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

class Decision(smach.State): # Define a state, outcomes and userdata hooks
    def __init__(self):
        smach.State.__init__(self, outcomes=['harvest', 'navigate','align','done','decide'], 
                            input_keys=['in_navigation','start_nav','row_side','plan','current_plan','previous_state',  'current_state'], output_keys=['out_navigation','start_nav','row_side','current_plan','previous_state',  'current_state'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.row_side_data = None
        self.plan = [['Navigate',[15,15,0]],['Harvest',[]],['Navigate',[25,15,0]],['Harvest',[]]]
        self.goal = [10,10,0]
        self.pub_goal = rospy.Publisher("/gplanner/goal", PoseStamped, queue_size=1)
        # self.plan_data =rospy.Subscriber("/plan_to_execute", Int64MultiArray, self.receive_plan)
        self.track_odometry =rospy.Subscriber("/odometry/filtered_map", Odometry, self.track_harvey)
        self.track_odometry_cartsian =rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
        self.current_location = [0,0,0]
        self.distance_to_goal = None
        self.nav_goal = rospy.Subscriber("/gplanner/navigation_status", Int8, self.track_navigation)


    def track_navigation(self,msg):
        self.nav_goal  = msg.data
        # return 
    def callback_start(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    def row_change(self,msg):
        self.row_side_data = msg.next_mode

    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))
    def mode_feedback(self, msg):
        self.mode_feedback = msg.next_mode
        # print('Mode Feedback',self.mode_feedback)
    def receive_global_plan_feedback(self,msg):
        self.plan_feedback = msg.data

    def track_harvey(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    def execute(self, userdata):
        userdata.current_state = 'DECISION'
        # userdata.plan = self.plan
        print((userdata.plan))
        print(self.mode_feedback_sub)
        # self.mode_change('START_NAVIGATION')

        while userdata.plan!=[]:
            userdata.previous_state.append(userdata.current_state)
            while self.mode_feedback != 'ASSISTED_ALIGNMENT':
                userdata.current_plan = userdata.plan[0]
                # print('current_plan',userdata.current_plan)
                print(userdata.current_plan, userdata.current_plan[0])
                if userdata.current_plan[0] =='Navigate':
                    return 'navigate'
                elif userdata.current_plan[0] =='Harvest':
                    self.mode_change('START_HARVESTING')
                    rospy.sleep(2)
                    if self.mode_feedback == 'START_HARVESTING':
                        return 'harvest'
                    # else:
                    #     return 'try'
                elif userdata.current_plan[0] == 'Align':
                    self.mode_change('ASSISTED_ALIGNMENT')
                    if self.mode_feedback == 'ASSISTED_ALIGNMENT':
                        return 'align' 
            
            # self.mode_change('ASSISTED_ALIGNMENT')
            if self.mode_feedback == 'ASSISTED_ALIGNMENT':
                return 'align' 
                
        return 'done'  
