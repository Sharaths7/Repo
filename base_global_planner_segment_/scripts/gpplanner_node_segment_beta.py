#!/usr/bin/env python
from importlib.resources import path
import rospy
import parameters as param
from hybrid_a_star_user_input import *
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from farming_bot_msgs.msg import (
    trajectory_interface_cartesianAction,
    trajectory_interface_cartesianActionGoal,
    TrajectoryInterfaceGoalStatus
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


def get_way_point_tree(waypoint):
    re_created_wp=[[w[0],w[1],w[2]] for w in waypoint]
    waypoints_tree=KDTree(re_created_wp)
    return waypoints_tree

def euclidean_distance(strt,end):
  # print('EUCLIDEAN ',np.sqrt((end[0]-strt[0])**2 + (end[1]-strt[1])**2))
  return np.sqrt((end[0]-strt[0])**2 + (end[1]-strt[1])**2)

def set_current_goal(msg):
  get_value.current_goal = msg




def feedback_to_reach_goal(msg):
  get_value.goal_reached = msg.current_goal
  get_value.distance_to_target = euclidean_distance(get_value.start,get_value.pub_path[-1])


def publish_pose_array(path):
    print('Published Trajectory')
    get_value.trajectory_pub = PoseArray()
    get_value.trajectory_pub.header.frame_id = 'map'
   
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
    
      
      get_value.trajectory_pub.poses.append(new_pose)
    print('number of poses',len(get_value.trajectory_pub.poses))
    publish_trajectory.publish(get_value.trajectory_pub)

def path_segment(path):
    path_sequence, seq = [],[]
    for i in range(len(path.directionlist)):
        if len(seq) == 0:
            seq.append((path.xlist[i],path.ylist[i],path.yawlist[i], path.directionlist[i]))
            direction = path.directionlist[i]
        elif path.directionlist[i]!=direction:
            direction = path.directionlist[i]
            path_sequence.append(seq)
            seq = []
            seq.append((path.xlist[i],path.ylist[i],path.yawlist[i], path.directionlist[i]))
        else:
            seq.append((path.xlist[i],path.ylist[i],path.yawlist[i], path.directionlist[i]))
            if i ==len(path.directionlist)-1:
                path_sequence.append(seq)
    return (path_sequence)

def set_start(msg):
  orientation_q = msg.pose.pose.orientation
  orientation_list = [orientation_q.x,
                      orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
  get_value.start = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
  if get_value.sequence_exist :
    publish_path_segment()
  if get_value.path_segment_publish == []:
    print('Waiting for Goal')


def set_map(msg):
  pass


def set_goal(msg):
  print('Goal Recieved')
  get_value.path, get_value.path_segment_publish, get_value.pub_path = [],[],[]
  # print(get_value.path_segment_publish,get_value.path,get_value.pub_path)
  global start, goal, goal_found,goal_reached, path_segment_publish
  orientation_q = msg.pose.orientation
  orientation_list = [orientation_q.x,
                      orientation_q.y, orientation_q.z, orientation_q.w]
  (_roll, _pitch, _yaw) = euler_from_quaternion(orientation_list)
  get_value.goal = [msg.pose.position.x, msg.pose.position.y, _yaw]
  ox, oy = [], []
  start_time = time.time()
  print(get_value.start,get_value.goal)
  get_value.path = hybrid_a_star_planning(
      get_value.start, get_value.goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
  get_value.compute_time = ("--- %s seconds ---" % (time.time() - start_time))
  get_value.path_segment_publish = path_segment(get_value.path)
  get_value.number_of_sequences  = len(get_value.path_segment_publish)
  if get_value.number_of_sequences>1:
    get_value.sequence_exist = True
  get_value.pub_path = get_value.path_segment_publish.pop(0)
  get_value.number_of_sequences-=1
  get_value.total_goals_to_reached = len(get_value.pub_path)
  publish_pose_array(get_value.pub_path)
  return get_value.path_segment_publish



def publish_path_segment():
    if get_value.number_of_sequences!=0:
      print('get_value.number_of_sequences:', get_value.number_of_sequences)
      print('Distance to Current sequence end point',euclidean_distance(get_value.start,get_value.pub_path[-1]))
      print('Current_goal',get_value.current_goal)
      for i in range(len(get_value.path_segment_publish)):
        print('euclidean_distance out of the condition',euclidean_distance(get_value.start,get_value.path_segment_publish[i][-1]))
        if abs(euclidean_distance(get_value.start,get_value.pub_path[-1]))<1.0: 
            print('euclidean_distance in the condition',euclidean_distance(get_value.start,get_value.pub_path[-1]))
            get_value.pub_path = get_value.path_segment_publish.pop(0)
            get_value.total_goals_to_reached = len(get_value.pub_path)
            get_value.number_of_sequences-=1
            print('get_value.number_of_sequences 2', get_value.number_of_sequences)
            print(get_value.pub_path)
            publish_pose_array(get_value.pub_path)


get_value = values()


if __name__ == "__main__":

  rospy.init_node("Global_planner_cartesian_interface")
  publish_trajectory = rospy.Publisher('/navigation/cartesian_trajectory_goal', PoseArray, queue_size=1)
  intermediate_goal = rospy.Subscriber("/move_base/current_goal", Pose, set_current_goal)
  publish_path = rospy.Publisher('/gplanner/path', Path, queue_size=1)
  start = rospy.Subscriber("/odometry/filtered_map", Odometry, set_start)
  map = rospy.Subscriber(
      "/move_base/global_costmap/costmap", OccupancyGrid, set_map)
  pathsequence_computed = rospy.Subscriber("/gplanner/goal", PoseStamped, set_goal)
  goal_achieved = rospy.Subscriber("/trajectory_interface_node/cartesian/trajectory_interface_goal_count", TrajectoryInterfaceGoalStatus, feedback_to_reach_goal)
  rospy.spin()
