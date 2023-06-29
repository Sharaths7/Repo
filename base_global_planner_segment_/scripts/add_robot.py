from farming_bot_msgs.srv import AddRobot
from farming_bot_msgs.srv import success_add_robot

# Service callback function
def handle_add_robot(req):
    robot_name = req.robot_name
    
    # Create state machine and motion planner instances
    state_machine = create_state_machine(robot_name)
    motion_planner = create_motion_planner(robot_name)
    
    # Store instances in dictionary
    state_machines[robot_name] = state_machine
    motion_planners[robot_name] = motion_planner
    
    # Return success status
    return SetBoolResponse(success=True)
    
# Service node
def service_node():
    rospy.init_node('service_node')
    
    # Create service
    add_robot_service = rospy.Service('add_robot', AddRobot, handle_add_robot)
    
    rospy.spin()