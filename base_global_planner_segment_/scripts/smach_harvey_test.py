#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import(
    String as str, 
    Bool, 
    Int8
) 

from my_msgs.msg import RobotPose
from my_msgs.srv import GetNextGoal, GetNextGoalRequest, GetNextGoalResponse
# from my_motion_planner import MotionPlanner


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

from decision_state import Decision
from navigation_state import Navigation
from align_state import Align
from harvest_state import Harvest


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




    def run(self):
        # Create state machine introspection server
        introspection_server = smach_ros.IntrospectionServer(
            'robot_%s_state_machine' % self.robot_name,
            self.state_machine,
            '/%s' % self.robot_name
        )
        
        # Start introspection server
        introspection_server.start()
        
        # Execute state machine
        outcome = self.state_machine.execute()
        
        # Stop introspection server
        introspection_server.stop()
        
        return outcome
    
    def get_next_goal_callback(self, req):
        # Get next goal from state machine
        next_goal = self.motion_planner.get_next_goal()
        
        # Return response with next goal
        return GetNextGoalResponse(next_goal)

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
    sm.userdata.current_state = None
    sm.userdata.plan_execution_status = None
    sm.userdata.previous_state = []
    sm.set_initial_state(['Decision'])
    with sm:
        smach.StateMachine.add('Decision', Decision(), # Add state and mapping for IO hooks
                transitions={'harvest':'Harvest','navigate':'Navigation','align': 'Align', 'decide':'Decision', 'done':'finished'},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation','start_nav':'sm_start_nav','row_side':'sm_row_side'})
        smach.StateMachine.add('Navigation', Navigation(), # Add state and mapping for IO hooks
                transitions={'running':'Navigation','align': 'Align', 'decide':'Decision'},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation','start_nav':'sm_start_nav','row_side':'sm_row_side'})
        smach.StateMachine.add('Harvest', Harvest(),      
                transitions={'done':'Decision',
                                'harvesting':'Harvest',},
                remapping={'in_navigation':'sm_navigation','out_navigation':'sm_navigation',
                            'in_harvest':'sm_harvest','out_harvest':'sm_harvest','start_nav':'sm_start_nav','row_side':'sm_row_side'})
        smach.StateMachine.add('Align', Align(),
                transitions={'align':'Align',
                                'harvesting':'Harvest',
                                'navigate': 'Navigation',
                                'decide': 'Decision'},
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
