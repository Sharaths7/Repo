#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String, Int64MultiArray
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

class Navigation(smach.State): # Define a state, outcomes and userdata hooks
    def __init__(self):
        smach.State.__init__(self, outcomes=['full', 'navigate','align','try','done'], 
                            input_keys=['in_navigation','start_nav','row_side','plan'], output_keys=['out_navigation','start_nav','row_side'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.row_side_data = None
        self.plan = [['Navigate',[15,15,0]],['Harvest',[]],['Navigate',[25,15,0]],['Harvest',[]]]
        self.goal = [10,10,0]
        self.pub_goal = rospy.Publisher("/gplanner/goal", PoseStamped, queue_size=1)
        self.plan_data =rospy.Subscriber("/plan_to_execute", Int64MultiArray, self.receive_plan)
        self.track_odometry =rospy.Subscriber("/odometry/filtered_map", Odometry, self.track_harvey)
        self.track_odometry_cartsian =rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)
        self.current_location = [0,0,0]
        self.distance_to_goal = None
        self.start_nav = False
    def callback_start(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    def row_change(self,msg):
        self.row_side_data = msg.data

    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))
    def mode_feedback(self, msg):
        self.mode_feedback = msg.next_mode
    def receive_plan(self,msg):
        self.plan = msg.data

    def track_harvey(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    def execute(self, userdata):
        # userdata.plan = self.plan
        print((userdata.plan))
        self.mode_change('START_NAVIGATION')

        while userdata.plan!=[]:
            plan = userdata.plan.pop(0)
            print(plan, plan[0])
            if plan[0] =='Navigate':
                rospy.loginfo(" I was here")
                # print(plan,plan[1])
                self.start_nav = True
                goal_to_achieve = convert_goal_pose(plan[1])
                rospy.sleep(2) 
                publish = True
                while self.start_nav!=False:
                    self.distance_to_goal = euclidean_distance(self.current_location,plan[1])

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
                        publish = True
                        return 'try'
            elif plan[0] =='Harvest':
                # self.mode_change('START_HARVESTING')
                    return 'full'
            elif plan[0] == 'Align':
                # self.mode_change('ASSISTED_ALIGNMENT')
                return 'align' 
            
        return 'try'    
    
class Harvest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','harvesting'],
                            input_keys=['in_navigation','in_harvest','start_nav','row_side'], 
                            output_keys=['out_navigation','in_harvest','out_harvest','start_nav','row_side'])
        # self.align_success
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        self.mode_feedback_sub =rospy.Subscriber("/machine_state/control_mode/mode_change_feedback", ModeSwitchRequestMessage, self.mode_feedback)

    def mode_feedback(self, msg):
        self.mode_feedback = msg.next_mode
    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))

    def execute(self, userdata):
        if userdata.in_harvest >= 50:
            userdata.start_nav = False
            userdata.in_harvest = 0
            rospy.loginfo(userdata.start_nav)  
            self.mode_change('STOP_HARVESTING')
            return 'done'
    
        else:
            userdata.out_harvest = userdata.in_harvest + 1
            rospy.sleep(0.5) 
            # userdata.out_navigation = userdata.in_navigation - 1
            rospy.loginfo('harvest: '+str(userdata.in_harvest) + ', navigation: ' + str(userdata.in_navigation))
            self.mode_change('START_HARVESTING')
            return 'harvesting'
        
class Align(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['align','harvesting'],
                            input_keys=['align','in_navigation','in_harvest','start_nav','row_side'], 
                            output_keys=['align','out_navigation','out_harvest','start_nav','row_side'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)


        
class Key_hole(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['align','harvesting'],
                            input_keys=['align','in_navigation','in_harvest','start_nav','row_side'], 
                            output_keys=['align','out_navigation','out_harvest','start_nav','row_side'])
        self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)





    def mode_change(self,value):
        self.request_change_publisher.publish(ModeSwitchRequestMessage(next_mode=value))

    def execute(self, userdata):
        userdata.align = 0
        if userdata.align>= 50:
            userdata.start_nav = False
            # self.mode_change('START_HARVESTING')
            rospy.loginfo(userdata.start_nav)  
            return 'harvesting'
        else:
            userdata.align =+ 1
            # userdata.out_navigation = userdata.in_navigation - 1
            rospy.loginfo('Align' + str(userdata.aligns))
            return 'align'
        
def init_harvey_userdata():
    #create SMACH state machine
    sm = smach.StateMachine(outcomes=['finished','aborted'])
    sm.userdata.sm_harvest = 0
    sm.userdata.sm_start_nav = False
    sm.userdata.sm_navigation = 0
    sm.userdata.sm_row_side = None
    sm.userdata.align = 0
    sm.userdata.plan = [['Harvest',[]],['Harvest',[]]]
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
                             'harvesting':'Harvest',},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation',
                           'in_harvest':'sm_harvest','out_harvest':'sm_harvest','start_nav':'sm_start_nav','row_side':'sm_row_side'})
    return sm
    
if __name__ == '__main__':
    rospy.init_node('harvey_smach_v1')
    rospy.sleep(0.5)          # Time to initialize to avoid missing log messages
    sm = init_harvey_userdata()  # Create state machine
    outcome = sm.execute()    # Execute state machine
    rospy.loginfo('Final outcome: '+outcome)
    rospy.spin()
  