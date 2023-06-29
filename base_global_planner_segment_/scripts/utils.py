#!/usr/bin/env python

from importlib.resources import path
from tokenize import String
from pyparsing import restOfLine
import rospy
from math import pi
import parameters as param
from hybrid_a_star_user_input import *
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray
)

def convert_goal_pose(goal):
    new_pose = PoseStamped()
    print(goal[0])
    new_pose.pose.position.x = goal[0]
    new_pose.pose.position.y = goal[1]
    new_pose.pose.position.z = 0
    roll = pitch = 0
    yaw_n = goal[2]
    quat = quaternion_from_euler(roll, pitch, yaw_n)
    new_pose.pose.orientation.x = quat[0]
    new_pose.pose.orientation.y = quat[1]
    new_pose.pose.orientation.z = quat[2]
    new_pose.pose.orientation.w = quat[3]
    return new_pose

    # pass
def invert_angle(angle):
  return (angle + pi) % (2 * pi);

def euclidean_distance(strt,end):
  # print('EUCLIDEAN ',np.sqrt((end[0]-strt[0])**2 + (end[1]-strt[1])**2))
  return np.sqrt((end[0]-strt[0])**2 + (end[1]-strt[1])**2)



def path_segment(path):
    path_sequence, seq = [],[]
    # print('Xlist',len(path.xlist))

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
    # print(len(path_sequence),path_sequence)
    return (path_sequence)


