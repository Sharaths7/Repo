#!/usr/bin/env python
import rospy
from hybrid_a_star_user_input import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from farming_bot_msgs.msg import (
    trajectory_interface_cartesianAction,
    trajectory_interface_cartesianActionGoal
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray
)
from nav_msgs.msg import (
    Path,
    Odometry,
    OccupancyGrid
)

start, goal = None, None
map_frame = "map"


def flatten(l):
    return [item for sublist in l for item in sublist]
def publish_pose_array(path):
    print('Published Trajectory')
    trajectory_pub = PoseArray()
    trajectory_pub.header.frame_id = 'map'
   
    for i in range(len(path)):
      
      new_pose = Pose()
      new_pose.position.x = path[i][0]
      new_pose.position.y = path[i][1]
      new_pose.position.z = 0
      roll = pitch = 0
      yaw_n = path[i][2]
      quat = quaternion_from_euler(roll, pitch, yaw_n)
      new_pose.orientation.x = quat[0]
      new_pose.orientation.y = quat[1]
      new_pose.orientation.z = quat[2]
      new_pose.orientation.w = quat[3]
    
      
      trajectory_pub.poses.append(new_pose)
    print('number of poses',len(trajectory_pub.poses))
    publish_trajectory.publish(trajectory_pub)




def publish_circuit(circuit):
    final_path = []
    # for path in circuit:

        
def set_start(msg):
  # print(msg)
  global start
  ox,oy = [],[]
  orientation_q = msg.pose.pose.orientation
  orientation_list = [orientation_q.x,
                      orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
  # roll, pitch, yaw =
  start = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
  goals = [[0,10,0],[10,10,90],[10,0,-90],[0,0,0]] 
  circuit = []
  for element in range (len(goals)):
    if element==1:
        path= hybrid_a_star_planning(start, goals[element], ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
        x = list(path.xlist)
        y = list(path.ylist)
        yaw = list(path.yawlist)
        circuit.append(list(zip(x, y, yaw)))
    else:
        if element<(len(goals)-1):
            path= hybrid_a_star_planning(goals[element], goals[element+1], ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
            x = list(path.xlist)
            y = list(path.ylist)
            yaw = list(path.yawlist)
            circuit.append(list(zip(x, y, yaw)))


    



  #   print(np.rad2deg(start[2]))
  # pass


def set_map(msg):
  # print(msg)
  pass


def set_goal(msg):
    pass
  # print(msg)
#   global start, goal
#   orientation_q = msg.pose.orientation
#   orientation_list = [orientation_q.x,
#                       orientation_q.y, orientation_q.z, orientation_q.w]
#   (_roll, _pitch, _yaw) = euler_from_quaternion(orientation_list)
#   goal = [msg.pose.position.x, msg.pose.position.y, _yaw]
#   print(goal)
#   ox, oy = [], []
#   start_time = time.time()

#   path = hybrid_a_star_planning(
#       start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

#   x = list(path.xlist)
#   y = list(path.ylist)
#   yaw = list(path.yawlist)

#   print("--- %s seconds ---" % (time.time() - start_time))
#   trajectory_pub = PoseArray()
#   trajectory_pub.header.frame_id = 'map'
#   # path_pub=Path()
#   # path_pub.header.frame_id='map'
#   # publish_trajectory(path)
#   for i in range(len(x)):

#     new_pose = Pose()
#     # new_pose.header.seq=i+1
#     # new_pose.header.frame_id='map'
#     new_pose.position.x = x[i]
#     new_pose.position.y = y[i]
#     # print(new_pose.pose.position.x,new_pose.pose.position.y)
#     new_pose.position.z = 0
#     roll = pitch = 0
#     yaw_n = yaw[i]
#     quat = quaternion_from_euler(roll, pitch, yaw_n)
#     new_pose.orientation.x = quat[0]
#     new_pose.orientation.y = quat[1]
#     new_pose.orientation.z = quat[2]
#     new_pose.orientation.w = quat[3]
#     # new_pose.pose.orientation=path_now[3]
#     trajectory_pub.poses.append(new_pose)
#     # path_pub.poses.append(new_pose)
#   # print(x)
#   # print(x,y,yaw)
#   publish_trajectory.publish(trajectory_pub)

  # publish_path.publish(path_pub)
#   end = list(zip(x, y, yaw))
#   print(len(end))


def feedback_cb(msg):
  print(msg)
  pass


def trajectory_Callback(msg):
  pass

  return action_goal


if __name__ == "__main__":
    rospy.init_node("Global_planner_circuit_cartesian_interface")
    publish_trajectory = rospy.Publisher('/navigation/cartesian_trajectory_goal', PoseArray, queue_size=1)

  # publish_path = rospy.Publisher('/gplanner/path', Path, queue_size=1)
#   publish_trajectory = rospy.Publisher(
#       '/navigation/cartesian_trajectory_goal', PoseArray, queue_size=1)
#   start = rospy.Subscriber("odometry/filtered_map", Odometry, set_start)
#   map = rospy.Subscriber(
#       "/move_base/global_costmap/costmap", OccupancyGrid, set_map)
#   goal = rospy.Subscriber("/gplanner/goal", PoseStamped, set_goal)

  # Print result
#   rospy.spin()
    start = [0,0,0]
    ox,oy = [],[]
    # orientation_q = msg.pose.pose.orientation
    # orientation_list = [orientation_q.x,
    #                     orientation_q.y, orientation_q.z, orientation_q.w]
    # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # # roll, pitch, yaw =
    # start = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
    goals = [[0,10,0],[10,10,90],[10,0,-90],[0,0,0]] 
    circuit = []
    for element in range (len(goals)):
        if element==0:
            path= hybrid_a_star_planning(start, goals[element], ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
            x = list(path.xlist)
            y = list(path.ylist)
            yaw = list(path.yawlist)
            circuit.append(list(zip(x, y, yaw)))
        else:
            if element<(len(goals)-1):
                path= hybrid_a_star_planning(goals[element], goals[element+1], ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
                x = list(path.xlist)
                y = list(path.ylist)
                yaw = list(path.yawlist)
                circuit.append(list(zip(x, y, yaw)))


    flat_list = []
    for sublist in circuit:
        for item in sublist:
            flat_list.append(item)
    final_path = flat_list
    publish_pose_array(final_path)
    rospy.spin()
        
