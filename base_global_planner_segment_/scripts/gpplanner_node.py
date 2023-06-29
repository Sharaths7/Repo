#!/usr/bin/env python
import rospy
import parameters as param
from hybrid_a_star_user_input import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from farming_bot_msgs.msg import (
    trajectory_interface_cartesianAction,
    trajectory_interface_cartesianActionGoal
)
from std_msgs.msg import String
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
def transform_vehicle(vehicle_point_object, next_state, angle_of_rotation):
  lst=[]
  # print('i came till here')
  for pt in vehicle_point_object.input_coordinates:
    new_x=(pt[0]-vehicle_point_object.center[0])*np.cos(angle_of_rotation) + (pt[1]-vehicle_point_object.center[1])*np.sin(angle_of_rotation)+next_state[0]
    new_y=-(pt[0]-vehicle_point_object.center[0])*np.sin(angle_of_rotation)+(pt[1]-vehicle_point_object.center[1])*np.cos(angle_of_rotation)+next_state[1]
    print(new_x, new_y)
    lst.append([new_x,new_y])
  vehicle_point_ret=param.vehicle_points(np.array(lst),next_state)
  # print(lst)
  return vehicle_point_ret



def row_change(msg):
  # print('Automated Row')
  get_value.row_side_data = msg.data
  # print(data)

class values:
  def __init__(self):    
    self.start = None
    self.goal = None
    self.map_frame = "map"
    self.path_segment_publish = []
    self.xlist = []
    self.ylist = []
    self.yawlist = []
    self.path = []
    self.goal_reached = None
    self.directions_list = []
    self.XY_GRID_RESOLUTION = param.XY_GRID_RESOLUTION
    self.YAW_GRID_RESOLUTION = param.YAW_GRID_RESOLUTION
    self.number_of_sequences = 0
    self.MOTION_RESOLUTION = param.MOTION_RESOLUTION
    self.goal_sequences = True
    self.N_STEER = param.N_STEER
    self.pub_path = []
    self.H_COST = param.XY_GRID_RESOLUTION
    self.VR = param.VR
    self.SB_COST = param.SB_COST
    self.BACK_COST = param.BACK_COST
    self.STEER_CHANGE_COST = param.STEER_CHANGE_COST
    self.STEER_COST = param.STEER_COST
    self.H_COST = param.H_COST
    self.trajectory_pub = PoseArray()
    self.trajectory_pub.header.frame_id = 'map'
    self.total_goals_to_reached = None
    self.sequence_exist = False
    self.current_goal = None
    self.compute_time = None
    self.row_points = None
    self.row_number = None
    self.row_side_data = None
    self.threshold_distance_to_goal = param.threshold_distance_to_goal


def invert_angle(angle):
  return (angle + pi) % (2 * pi)

def set_start(msg):
  # print(msg)
  global start
  ox, oy = [], []
  orientation_q = msg.pose.pose.orientation
  orientation_list = [orientation_q.x,
                      orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
  # roll, pitch, yaw =
  get_value.start = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
  #   print(np.rad2deg(start[2]))
  # pass
  if get_value.row_side_data!=None:
    vehicle_points = param.vehicle_pt_obj_actual
    get_value.row_points = transform_vehicle(vehicle_points, get_value.start,get_value.start[2])
    row_goal =  [0,0,0]
    print('get_value.start',get_value.start)
    print(get_value.row_side_data)
    if get_value.row_side_data  == 'Right':
      row_goal[0] =  get_value.row_points.input_coordinates[0][0]
      row_goal[1] =  get_value.row_points.input_coordinates[0][1]
      row_goal[2] =  invert_angle(get_value.start[2])
      print("row_goal",row_goal)
      get_value.row_side_data  = None
      get_value.path = hybrid_a_star_planning(get_value.start, row_goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
      x = list(get_value.path.xlist)
      y = list(get_value.path.ylist)
      yaw = list(get_value.path.yawlist)

      # print("--- %s seconds ---" % (time.time() - start_time))
      trajectory_pub = PoseArray()
      trajectory_pub.header.frame_id = 'map'
      # path_pub=Path()
      # path_pub.header.frame_id='map'
      # publish_trajectory(path)
      for i in range(len(x)):

        new_pose = Pose()
        # new_pose.header.seq=i+1
        # new_pose.header.frame_id='map'
        new_pose.position.x = x[i]
        new_pose.position.y = y[i]
        # print(new_pose.pose.position.x,new_pose.pose.position.y)
        new_pose.position.z = 0
        roll = pitch = 0
        yaw_n = yaw[i]
        quat = quaternion_from_euler(roll, pitch, yaw_n)
        new_pose.orientation.x = quat[0]
        new_pose.orientation.y = quat[1]
        new_pose.orientation.z = quat[2]
        new_pose.orientation.w = quat[3]
        # new_pose.pose.orientation=path_now[3]
        trajectory_pub.poses.append(new_pose)
        # path_pub.poses.append(new_pose)
      # print(x)
      # print(x,y,yaw)
      publish_trajectory.publish(trajectory_pub)

      # get_value.compute_time = ("--- %s seconds ---" % (time.time() - start_time))
      # print('After Computing:' , len(get_value.path.xlist))
      # x = list(path.xlist)sequence_exist
      # get_value.path_segment_publish = path_segment(get_value.path)
      # get_value.number_of_sequences  = len(get_value.path_segment_publish)
      # # if get_value.number_of_sequences == 1:
      # #   get_value.goal_sequences = False
      # if get_value.number_of_sequences>1:
      #   get_value.sequence_exist = True
      # get_value.pub_path = get_value.path_segment_publish.pop(0)
      # get_value.number_of_sequences-=1
      # get_value.total_goals_to_reached = len(get_value.pub_path)
      # publish_pose_array(get_value.pub_path)

      # print('Awesome')

      # pass
    
    elif get_value.row_side_data  == 'Left':
      row_goal[0] =  get_value.row_points.input_coordinates[1][0]
      row_goal[1] =  get_value.row_points.input_coordinates[1][1]
      row_goal[2] =  invert_angle(get_value.start[2])
      print("row_goal",row_goal)

      get_value.path = hybrid_a_star_planning(get_value.start, row_goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
      x = list(get_value.path.xlist)
      y = list(get_value.path.ylist)
      yaw = list(get_value.path.yawlist)

      # print("--- %s seconds ---" % (time.time() - start_time))
      trajectory_pub = PoseArray()
      trajectory_pub.header.frame_id = 'map'
      # path_pub=Path()
      # path_pub.header.frame_id='map'
      # publish_trajectory(path)
      for i in range(len(x)):

        new_pose = Pose()
        # new_pose.header.seq=i+1
        # new_pose.header.frame_id='map'
        new_pose.position.x = x[i]
        new_pose.position.y = y[i]
        # print(new_pose.pose.position.x,new_pose.pose.position.y)
        new_pose.position.z = 0
        roll = pitch = 0
        yaw_n = yaw[i]
        quat = quaternion_from_euler(roll, pitch, yaw_n)
        new_pose.orientation.x = quat[0]
        new_pose.orientation.y = quat[1]
        new_pose.orientation.z = quat[2]
        new_pose.orientation.w = quat[3]
        # new_pose.pose.orientation=path_now[3]
        trajectory_pub.poses.append(new_pose)
        # path_pub.poses.append(new_pose)
      # print(x)
      # print(x,y,yaw)
      publish_trajectory.publish(trajectory_pub)

      # get_value.compute_time = ("--- %s seconds ---" % (time.time() - start_time))
      # print('After Computing:' , len(get_value.path.xlist))
      # x = list(path.xlist)sequence_exist
      # get_value.path_segment_publish = path_segment(get_value.path)
      # get_value.number_of_sequences  = len(get_value.path_segment_publish)
      # # if get_value.number_of_sequences == 1:
      # #   get_value.goal_sequences = False
      # if get_value.number_of_sequences>1:
      #   get_value.sequence_exist = True
      # get_value.pub_path = get_value.path_segment_publish.pop(0)
      # get_value.number_of_sequences-=1
      # get_value.total_goals_to_reached = len(get_value.pub_path)
      # get_value.row_side_data  = None
      # publish_pose_array(get_value.pub_path)



def set_map(msg):
  # print(msg)
  pass


def set_goal(msg):
  # print(msg)
  global start, goal
  orientation_q = msg.pose.orientation
  orientation_list = [orientation_q.x,
                      orientation_q.y, orientation_q.z, orientation_q.w]
  (_roll, _pitch, _yaw) = euler_from_quaternion(orientation_list)
  goal = [msg.pose.position.x, msg.pose.position.y, _yaw]
  print(goal)
  ox, oy = [], []
  start_time = time.time()

  path = hybrid_a_star_planning(
      start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

  x = list(path.xlist)
  y = list(path.ylist)
  yaw = list(path.yawlist)

  print("--- %s seconds ---" % (time.time() - start_time))
  trajectory_pub = PoseArray()
  trajectory_pub.header.frame_id = 'map'
  # path_pub=Path()
  # path_pub.header.frame_id='map'
  # publish_trajectory(path)
  for i in range(len(x)):

    new_pose = Pose()
    # new_pose.header.seq=i+1
    # new_pose.header.frame_id='map'
    new_pose.position.x = x[i]
    new_pose.position.y = y[i]
    # print(new_pose.pose.position.x,new_pose.pose.position.y)
    new_pose.position.z = 0
    roll = pitch = 0
    yaw_n = yaw[i]
    quat = quaternion_from_euler(roll, pitch, yaw_n)
    new_pose.orientation.x = quat[0]
    new_pose.orientation.y = quat[1]
    new_pose.orientation.z = quat[2]
    new_pose.orientation.w = quat[3]
    # new_pose.pose.orientation=path_now[3]
    trajectory_pub.poses.append(new_pose)
    # path_pub.poses.append(new_pose)
  # print(x)
  # print(x,y,yaw)
  publish_trajectory.publish(trajectory_pub)

  # publish_path.publish(path_pub)
  end = list(zip(x, y, yaw))
  print(len(end))


def feedback_cb(msg):
  print(msg)
  pass


def trajectory_Callback(msg):
  pass

  return action_goal


get_value = values()

if __name__ == "__main__":

  rospy.init_node("Global_planner_cartesian_interface")

  # publish_path = rospy.Publisher('/gplanner/path', Path, queue_size=1)
  publish_trajectory = rospy.Publisher(
      '/navigation/cartesian_trajectory_goal', PoseArray, queue_size=1)
  start = rospy.Subscriber("odometry/filtered_map", Odometry, set_start)
  map = rospy.Subscriber(
      "/move_base/global_costmap/costmap", OccupancyGrid, set_map)
  goal = rospy.Subscriber("/gplanner/goal", PoseStamped, set_goal)
  row = rospy.Subscriber("/side", String, row_change)

  # Print result
  rospy.spin()
