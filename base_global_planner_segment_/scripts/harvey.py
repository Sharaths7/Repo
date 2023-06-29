import matplotlib.pyplot as plt
from math import sqrt, cos, sin, tan, pi

WB = 3#7.  # distance from rear to front wheel
W =  2#2.48568  # width of a vehicle
LF =  3  # distance from rear to vehicle front end
LB =  3  # distance from rear to vehicle back end
MAX_STEER = 0.6  # [rad] maximum steering angle of the vehicle
print ("Vehicle Parameters:", "WB :", WB, "W: ",W, "LF: ",LF , "LB: ", LB )
#Create A buffer region (Circular) around the vehicle to check for collison avoidance
WBUBBLE_DIST = (LF - LB) / 2.0 
WBUBBLE_R = sqrt(((LF + LB) / 2.0)**2 + 1) #Radius to search poinbts around the waypoints
#for collision avoidance 

# vehicle rectangle verticles;These would be the augmented points to consider 
# vehcile's dimension during plotting the harvey along the occupancy grid and waypoints
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W / 2, -W / 2, -W / 2, W / 2, W / 2]


def check_harvey_collision(xlist, ylist, yawlist, ox, oy, kdtree):
    for x, y, yaw in zip(xlist, ylist, yawlist):
        cx = x + WBUBBLE_DIST * cos(yaw)
        cy = y + WBUBBLE_DIST * sin(yaw)

        ids = kdtree.search_in_distance([cx, cy], WBUBBLE_R)

        if not ids:
            continue

        if not rectangle_check(x, y, yaw,
                               [ox[i] for i in ids], [oy[i] for i in ids]):
            return False  # collision

    return True  # no collision


def rectangle_check(x, y, yaw, ox, oy):
    # transform obstacles to base link frame-coordinates
    c, s = cos(-yaw), sin(-yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        rx = c * tx - s * ty
        ry = s * tx + c * ty
        # print(x,y,tx,ty,rx,ry,LF,-LB,W/2.0,-W/2.0)
        if not (rx > LF or rx < -LB or ry > W / 2.0 or ry < -W / 2.0):
            return False  # no collision
    # print("collision")
    return True  # collision

##Plot the orientation of the harvey while traversing
##through the waypoints
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)
        # plt.plot(x, y)

##A function to plot to the harvey'pose and orientation given x,y,yaw
def plot_harvey(x, y, yaw):
    harvey_color = '-k'
    c, s = cos(yaw), sin(yaw)

    harvey_outline_x, harvey_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        tx = c * rx - s * ry + x
        ty = s * rx + c * ry + y
        harvey_outline_x.append(tx)
        harvey_outline_y.append(ty)

    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(harvey_outline_x, harvey_outline_y, harvey_color)


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi

#Update Finction to compute the next pose
def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw += pi_2_pi(distance * tan(steer) / L)  # distance/2

    return x, y, yaw
