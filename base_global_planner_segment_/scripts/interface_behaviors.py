#!/usr/bin/env python
from importlib.resources import path
# from tokenize import String
import time
from pyparsing import restOfLine
import rospy
from math import pi
from utils import *
import parameters as param
from hybrid_a_star_user_input import *
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from farming_bot_msgs.msg import (
    trajectory_interface_cartesianAction,
    trajectory_interface_cartesianActionGoal,
    TrajectoryInterfaceGoalStatus,
    ModeSwitchRequestMessage
)
from std_msgs.msg import (
    String,
    Int8
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

class interface ():
    def __init__(self):
        # self.request_change_publisher = rospy.Publisher("/machine_inputs/control_mode/request_mode_change", ModeSwitchRequestMessage, queue_size=10)
        # self.MODE_CHANGE_TOPIC = '/machine_inputs/control_mode/request_mode_change'
        self.start = rospy.Subscriber("/odometry/filtered_map", Odometry, self.callback_start)
        self.goal = rospy.Subscriber("/gplanner/goal", PoseStamped, self.callback_goal)
        self.row_change_msg = rospy.Subscriber("/side", String, self.callback_row_change_data)
        self.ox = []
        self.oy = []
        self.row_side_data = None
        self.path  = []
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
        self.vehicle_points = param.vehicle_pt_obj_actual
        self.row_points = []
        self.row_goal =  [0,0,0]
        self.is_goal_set = None
        self.current_target_goal = None
        self.key_hole_success = None
        self.current_state = "NAVIGATION"
        self.alignment_success = None
        self.start_harvest = None
        self.navigation_sequence_feedback = None
        self.navigation_status_feedback = None
        self.namespace = None

    def publish_path_segment(self):
        print('self.number_of_sequences in publish callback', self.number_of_sequences)
        publish_feedback_nav_sequence.publish(self.number_of_sequences)
        if self.number_of_sequences!=0:
            # print('self.number_of_sequences:', self.number_of_sequences)
            # print('Distance to Current sequence end point',euclidean_distance(self.start,self.pub_path[-1])) n
            # print('Current_goal',self.current_goal)
            for i in range(len(self.path_segment_publish)):
                # print('self.goal_reached',self.goal_reached,'iter',i,'next sequences',len(self.path_segment_publish[0]),'msg.number_of_goals',self.total_goals_to_reached)
                # print('euclidean_distance out of the condition',euclidean_distance(self.start,self.path_segment_publish[i][-1]))    
                if abs(euclidean_distance(self.start,self.pub_path[-1]))<self.threshold_distance_to_goal: 
                # if self.goal_reached == self.total_goals_to_reached :
                    # print('euclidean_distance in the condition',euclidean_distance(self.start,self.pub_path[-1]))
                    self.pub_path = self.path_segment_publish.pop(0)
                    self.total_goals_to_reached = len(self.pub_path)
                    self.number_of_sequences-=1
                    print(self.pub_path)
                    self.publish_pose_array(self.pub_path)
        else:
            print('self.number_of_sequences 3', self.number_of_sequences)
            publish_feedback_nav_sequence.publish(self.number_of_sequences)

    def path_segment(self,path):
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

    def transform_vehicle(self, next_state, angle_of_rotation):
        lst=[]
        for pt in self.vehicle_points.input_coordinates:
            new_x=(pt[0]-self.vehicle_points.center[0])*np.cos(angle_of_rotation) + (pt[1]-self.vehicle_points.center[1])*np.sin(angle_of_rotation)+next_state[0]
            new_y=-(pt[0]-self.vehicle_points.center[0])*np.sin(angle_of_rotation)+(pt[1]-self.vehicle_points.center[1])*np.cos(angle_of_rotation)+next_state[1]
            # print(new_x, new_y)
            lst.append([new_x,new_y])
        self.row_points=param.vehicle_points(np.array(lst),next_state)
  
    def callback_start(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.start = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        self.transform_row_points()
        if self.sequence_exist :
            self.publish_path_segment()
        else:
            publish_feedback_nav_sequence.publish(0)
        if self.path_segment_publish == []:
            publish_feedback_nav_sequence.publish(0)
            self.sequence_exist = False
            if self.is_goal_set != True:
                rospy.loginfo("Waiting for New Goal:")
            # print('Waiting for Goal')
        if self.row_side_data!=None:
            self.row_change()
            

    def callback_goal(self, msg):
        print('Goal Recieved')
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (_roll, _pitch, _yaw) = euler_from_quaternion(orientation_list)
        self.goal = [msg.pose.position.x, msg.pose.position.y, _yaw] 
        self.current_target_goal = [msg.pose.position.x, msg.pose.position.y, _yaw]
        self.is_goal_set = True
        rospy.loginfo("Current Start Position (%1.3f, %1.3f, %1.3f)", self.start[0], self.start[1], self.start[2])
        rospy.loginfo("Current Goal Position (%1.3f, %1.3f, %1.3f)", self.goal[0], self.goal[1], self.goal[2])

        # print('Amazing')
        self.planning(self.start,self.goal)


    def callback_row_change_data(self,msg):
        # rospy.loginfo("Current Position (%s)", msg.data)
        self.row_side_data = msg.data
    def row_change(self):
        self.current_target_goal = None
        if self.row_side_data  == 'Right':
            self.row_goal =  [self.row_points.input_coordinates[0][0],self.row_points.input_coordinates[0][1], invert_angle(self.start[2])]
            rospy.loginfo("Current Start Position (%1.3f, %1.3f, %1.3f)", self.start[0], self.start[1], self.start[2])

            rospy.loginfo("Location to reach on the Right: (%1.3f, %1.3f, %1.3f)", self.row_goal[0], self.row_goal[1], self.row_goal[2])
            
            # print(self.row_goal)
        elif self.row_side_data  == 'Left':
            self.row_goal =  [self.row_points.input_coordinates[1][0],self.row_points.input_coordinates[1][1], invert_angle(self.start[2])]
            rospy.loginfo("Current Start Position (%1.3f, %1.3f, %1.3f)", self.start[0], self.start[1], self.start[2])
            rospy.loginfo("Location to reach on the Left: (%1.3f, %1.3f, %1.3f)", self.row_goal[0], self.row_goal[1], self.row_goal[2])

        self.current_target_goal = self.row_goal
        print('self.current_target_goal ', self.current_target_goal )
        self.planning(self.start, self.row_goal)
        self.row_side_data  = None

    def planning(self, start, goal):
        self.goal_reached = False
        start_time = time.time()
        # print('wows')
        self.path = hybrid_a_star_planning(start,goal,self.ox,self.oy, self.XY_GRID_RESOLUTION, self.YAW_GRID_RESOLUTION)
        self.compute_time = ("--- %s seconds ---" % (time.time() - start_time))
        print('After Computing found :' , len(self.path.xlist), ' Target Poses')
        # x = list(path.xlist)sequence_exist
        self.path_segment_publish = path_segment(self.path)
        self.number_of_sequences  = len(self.path_segment_publish)
        # if self.number_of_sequences == 1:
        #   self.goal_sequences = False
        if self.number_of_sequences>1:
            self.sequence_exist = True
        self.pub_path = self.path_segment_publish.pop(0)
        publish_feedback_nav_sequence.publish(self.number_of_sequences)
        self.number_of_sequences-=1
        self.total_goals_to_reached = len(self.pub_path)
        self.publish_pose_array(self.pub_path)
        # return path

    def transform_row_points(self):
        self.transform_vehicle(self.start,self.start[2])

    # def row_change(self):
    #     self.path = hybrid_a_star_planning(self.start,self.goal,self.ox,self.oy, self.XY_GRID_RESOLUTION, self.YAW_GRID_RESOLUTION)


    def publish_pose_array(self,path):
        self.trajectory_pub = PoseArray()
        self.trajectory_pub.header.frame_id = 'map'
    
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
            self.trajectory_pub.poses.append(new_pose)
        publish_trajectory.publish(self.trajectory_pub)


if __name__ == "__main__":
  rospy.init_node("behavior_interface")
  publish_trajectory = rospy.Publisher('/navigation/cartesian_trajectory_goal', PoseArray, queue_size=1)
  publish_feedback_nav_sequence = rospy.Publisher('/gplanner/navigation_sequence', Int8, queue_size=1)
  publish_feedback_nav_status = rospy.Publisher('/gplanner/navigation_status', String, queue_size=1)
  
  interface()
  rospy.spin()
