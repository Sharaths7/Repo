import numpy as np
XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(30.0)  # [rad]
MOTION_RESOLUTION = 3.0  # [m] path interporate resolution
N_STEER = 20.0  # number of steer command
# H_COST = 1.0
VR = 5.0  # robot radius
SB_COST = 1.0  # switch back penalty cost
BACK_COST = 3.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
H_COST = 5.0  # Heuristic cost
row_y_offset = 1.0
show_animation = True
threshold_distance_to_goal = 1.0
# vehicle_point_object.input_coordinates
class vehicle_points():
  def __init__(self,input_coordinates,center):
    self.input_coordinates=input_coordinates
    self.center=center

vehicle_pt_obj_actual = vehicle_points( np.array([[-1.5325, 0.85], #Right
                                                 [-1.5325,-0.85]]), #Left

                                                  [0,0] )