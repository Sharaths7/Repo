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
        smach.State.__init__(self, outcomes=['full', 'navigate','align','try','done'], 
                            input_keys=['in_navigation','start_nav','row_side','plan','current_plan'], output_keys=['out_navigation','start_nav','row_side','current_plan'])
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
        # userdata.plan = self.plan
        print((userdata.plan))
        print(self.mode_feedback_sub)
        # self.mode_change('START_NAVIGATION')

        while userdata.plan!=[]:
            while self.mode_feedback != 'ASSISTED_ALIGNMENT':
                plan = userdata.plan[0]
                userdata.current_plan = plan
                # print('current_plan',userdata.current_plan)
                print(plan, plan[0])
                if plan[0] =='Navigate':
                    # rospy.loginfo(" Plan: Navigate to : ", plan[0])
                    # print(plan,plan[1])
                    self.start_nav = True
                    goal_to_achieve = convert_goal_pose(plan[1])
                    rospy.sleep(2) 
                    publish = True
                    while self.start_nav!=False:
                        if self.mode_feedback != 'ASSISTED_ALIGNMENT':
                            self.distance_to_goal = euclidean_distance(self.current_location,plan[1])
                            if publish == True:
                                print(userdata.plan)
                                print('Publishing Goal and state')
                                self.pub_goal.publish(goal_to_achieve)
                                self.mode_change('START_NAVIGATION')
                                print(self.distance_to_goal)
                                print(param.threshold_distance_to_goal)
                                publish = False

                            # self.start_nav = False
                            # return 'try'
                            if self.distance_to_goal < param.threshold_distance_to_goal:
                                self.mode_change('ASSISTED_ALIGNMENT')
                                executed_plan = userdata.plan.pop(0)
                                self.start_nav = False
                                publish = True
                                return 'align'
                        else:
                            # userdata.plan[0:0] = userdata.current_plan
                            print(userdata.plan)
                            self.mode_change('ASSISTED_ALIGNMENT')
                            # userdata.plan[0:0] = userdata.current_plan
                            if self.mode_feedback == 'ASSISTED_ALIGNMENT':
                                return 'align' 
                elif plan[0] =='Harvest':
                    self.mode_change('START_HARVESTING')
                    rospy.sleep(2)
                    if self.mode_feedback == 'START_HARVESTING':
                        return 'full'
                    # else:
                    #     return 'try'
                elif plan[0] == 'Align':
                    self.mode_change('ASSISTED_ALIGNMENT')
                    if self.mode_feedback == 'ASSISTED_ALIGNMENT':
                        return 'align' 
            
            self.mode_change('ASSISTED_ALIGNMENT')
            if self.mode_feedback == 'ASSISTED_ALIGNMENT':
                return 'align' 
                
        return 'done'  


    
class Harvest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','harvesting'],
                            input_keys=['in_navigation','in_harvest','start_nav','row_side','current_plan'], 
                            output_keys=['out_navigation','in_harvest','out_harvest','start_nav','row_side','current_plan'])
        # self.align_success
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
        self.start_harvest = False
        self.harvest_execute = None 
        self.current_location = [0,0,0]
        self.track_odometry_cartsian =rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.publish = True

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
        while self.mode_feedback != 'ASSISTED_ALIGNMENT':
        
            self.harvest_execute = userdata.current_plan
            # rospy.loginfo(" Plan: Harvest till : ", self.harvest_execute[0])
            self.start_harvest = True
            goal_to_achieve = convert_goal_pose(self.harvest_execute[1])


            while self.mode_feedback!='False':
                self.distance_to_goal = euclidean_distance(self.current_location,self.harvest_execute[1])
                print(self.distance_to_goal)
                if self.distance_to_goal < param.threshold_distance_to_goal:
                    self.mode_change('STOP_HARVESTING')
                    self.start_harvest = False
                    publish = True
                    return 'done'
                return 'harvesting'

        userdata.plan[0:0] = userdata.current_plan
        self.mode_change('ASSISTED_ALIGNMENT')
        if self.mode_feedback == 'ASSISTED_ALIGNMENT':
            return 'align'

        
        
class Align(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['align','harvesting','try'],
                            input_keys=['align','in_navigation','in_harvest','start_nav','row_side', 'current_plan'], 
                            output_keys=['align','out_navigation','out_harvest','start_nav','row_side','current_plan'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))
    def mode_feedback(self, msg):
            self.mode_feedback = msg.next_mode
    def execute(self, userdata):
        if self.mode_feedback == 'ASSISTED_ALIGNMENT':
            return 'align'
        else:
            if self.mode_feedback == 'START_NAVIGATION':
                self.mode_change('NAVIGATION')
                return 'try'
            if self.mode_feedback == 'START_HARVESTING':
                return 'harvesting'
        return 'align'


        
def init_harvey_userdata(plan):
    #create SMACH state machine
    sm = smach.StateMachine(outcomes=['finished','aborted'])
    sm.userdata.sm_harvest = 0
    sm.userdata.sm_start_nav = False
    sm.userdata.sm_navigation = 0
    sm.userdata.sm_row_side = None
    sm.userdata.align = 0
    sm.userdata.plan = plan
    sm.userdata.current_plan = None
    sm.set_initial_state(['Navigation'])
    with sm:
        smach.StateMachine.add('Navigation', Navigation(), # Add state and mapping for IO hooks
                transitions={'full':'Harvest','navigate':'Navigation','align': 'Align', 'try':'Navigation', 'done':'finished'},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation','start_nav':'sm_start_nav','row_side':'sm_row_side'})
        smach.StateMachine.add('Harvest', Harvest(),      
                transitions={'done':'Navigation',
                                'harvesting':'Harvest',},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation',
                            'in_harvest':'sm_harvest','out_harvest':'sm_harvest','start_nav':'sm_start_nav','row_side':'sm_row_side'})
        smach.StateMachine.add('Align', Align(),
                transitions={'align':'Align',
                                'harvesting':'Harvest',
                                'try': 'Navigation'},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation',
                            'in_harvest':'sm_harvest','out_harvest':'sm_harvest','start_nav':'sm_start_nav','row_side':'sm_row_side'})
    return sm

def plan_callback(msg):
    plan = literal_eval(msg.data)
    sm = init_harvey_userdata(plan)  # Create state machine
    outcome = sm.execute()    # Execute state machine
    rospy.loginfo('Final outcome: '+outcome)
    return outcome




if __name__ == '__main__':
    rospy.init_node('harvey_smach_v2')
    rospy.sleep(0.5)          # Time to initialize to avoid missing log messages
    outcome = rospy.Subscriber("/plan", str, plan_callback, queue_size=10)
    # start = rospy.Subscriber("/odometry/filtered_map", Odometry, callback_start)
    rospy.spin()
