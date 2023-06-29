# Global Planner Hybrid A*

## Run

```bash
cd FieldRobots-test/navigation_ws
source devel/setup.bash
rosrun base_trajectory_interface gpplanner_node.py

#In RViz provide  2D Nav Goal @ Topic : /gplanner/goal
# Vizulaize the trajectory @ /navigation/cartesian_trajectory_goal

```

Run Test file:

```bash
python hybrid_a_star_user_input.py
```

Run the Jupyter Notebook

```bash
hya*.ipynb
```

## Dependencies

```latex
numpy
scipy.spatial
matplotlib
time
math
heapq
Plotly
```

## Scope and Objectives

The points listed below describe the key requirements regarding this algorithm:

1. Planning based on a local obstacle map
2. Ensuring real-time capability
3. Performing analysis and evaluation of results

The input to the path planning algorithm is a grid-based binary obstacle map. The produced paths must be continuous and need to be based on a model of the vehicle. In order to use the algorithm in a system, it needs to re-plan continuously and perform collision checking.

Fully autonomous robots need to reason, perceive, and control themselves adequately, planning plays a key role. Planning consumes the robot’s knowledge about the world (localization) to provide its actuators (controllers) with appropriate actions (references) to perform the task on the go. Machine learning may still be one of the largest areas in artificial intelligence, but planning can be seen as a necessary complement to it, as in the future decisions need to be formed autonomously based on the learned.

### Flowchart

![images/Untitled.png](images/Untitled.png)

## Planning

The autonomous vehicle is reduced to a 2D space of a single plane - where the vehicle is restricted by some constraints. The aim of solving this NP-hardness is not to find one solution that connects the start point and the goal point, but the optimal solution with the minimum distance and the smoothest maneuvers without hitting any known obstacles.

The *global planner* uses a priori information of the environment to create the best possible path

## Heuristics

The goal is to rollout drivable solutions that are approaching the optimum, it is important to make use of A* being an informed search, implementing heuristics allowing the algorithm converges quickly towards the solution. HA* is using estimates from two heuristics. As both of the heuristics are admissible the maximum of the two is chosen for any given state. 

The two heuristics capture very different parts of the problem:

- The constrained heuristic incorporates the restrictions of the vehicle, ignoring the environment
- The unconstrained heuristic disregards the vehicle constraints and only accounts for obstacles

### Constrained heuristic: Reeds Shepp's path

The constrained heuristic takes the characteristics of the vehicle into account while neglecting the environment. Suitable candidates are either Dubins or Reeds-Shepp curves. Since this heuristic takes the current heading as well as the turning radius into account it ensures, that the vehicle approaches the goal with the correct heading. This is especially important when Harvey gets closer to the goal. For performance reasons, this heuristic can be pre-computed and stored in a lookup table. This is possible since it ignores obstacles and hence does not require any environmental information. As it only improves the performance and not the quality of the solution a lookup table has not been implemented. The only difference in comparison to the Dubins path is that travel in the reverse direction is now allowed. The criterion which is the distance traveled by the center of the rear axle is optimized. The shortest path is equivalent to the path that takes minimum time, as for the Dubins path. 

![images/Untitled%201.png](images/Untitled%201.png)

![images/Untitled%202.png](images/Untitled%202.png)

### Unconstrained heuristic: DP Planning

A* Search is a refinement of Dijkstra’s work. While Dijkstra associated a `cost-so-far g` with each vertex of a graph to determine the next vertex to expand, A* enhances the algorithm by the use of a heuristic, allowing for much more rapid convergence under certain conditions, while still ensuring its optimal. The heuristic h(x) is the cost-to-come, based on an estimate of the cost from state x to the `goal state xg.` Just as Dijkstra’s algorithm A* also starts with `O` and `C` being the open and closed list respectively. Just as Dijkstra’s algorithm A* the starts with an empty `closed set C` `open set O`. Here the heuristic estimate comes into play so that the cost for a state `x` is thus `f(x) = g(x)+h(x)` by which the priority queue will be sorted. A standard heuristic estimate that can speed up search significantly, while maintaining admissibility and thus optimal, is the Euclidean norm for two-dimensional problems.

**Why can’t you use A* to navigate the system instead of using a complicated algorithm?**

The answer is the feasibility of Harvey dynamics. It can’t take turns abruptly as the path is returned by A*. Considering the path, it is not at all feasible for Harvey like a robot to travel. The A* search is also a discrete spaced algorithm while we need a continuous spaced search method. So, we need an algorithm that must impart the constraint of the motion of the vehicle as well. Hence, we are stepping into Hybrid A*.

## Hybrid A-Star

The hybrid A* algorithm behaves like the A* algorithm. The key difference is, that state transitions happen in continuous rather than discrete space. One of the largest drawbacks of the previous approaches for path planning of non-holonomic robots is that the resulting paths are discrete and thus often not executable as changes of direction are sudden rather than smooth.

While the hybrid A* search implicitly builds the graph on a discretized grid, vertices can reach any continuous point on the grid. As a continuous search space would not be finite a discretization in the form of grid cells is taken, limiting the growth of the graph. Since transitions from vertex to vertex have no predefined form it is easy to incorporate the non-holonomic nature in the state transition. The search space is usually three-dimensional so that the state space X consists of x, y, and θ, creating a discretized cuboid with the base representing the x, y position, and the height of the heading theta of a vertex.

## Collision Avoidance

The vehicle constraints such as Distance from rear to front wheel `WB`, Width of the Harvey `W`, Distance from the rear to the front end, and distance from rear to the back end `LF`and`LB`.Using such constraints a rectangle is created and is used to check for collision points based on a KDTree closest obstacles search from the waypoints using a specific lookup radius. All the queried points are then scrutinized for falling under with the rectangle after being converted into the vehicle's local coordinate frame. 

```python
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
```

## Tuneable Parameters

Hybrid A-star could be tuned using various cost parameters and the vehicle dynamic model. To obtain a path far away from obstacle we could increase the vehicle radius (`VR`) that would alter the collision avoidance module to check for a bigger radius from the obstacle point. Altering vehicle dynamics considering the vehicle model, we would be able to alter the dimension of the rectangle window which is used to check collision before rolling out a safe trajectory. 

### parameters.py

```python
import numpy as np
XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(30.0)  # [rad]
MOTION_RESOLUTION = 1.0  # [m] path interporate resolution
N_STEER = 20.0  # number of steer command
H_COST = 1.0
VR = 5.0  # robot radius
SB_COST = 10.0  # switch back penalty cost
BACK_COST = 9.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
H_COST = 5.0  # Heuristic cost

show_animation = True
```

### harvey[.py](http://car.py) parameters

```python
WB = 7.  # distance from rear to front wheel
W =  2.48568  # width of a vehicle
LF =  7.12593  # distance from rear to vehicle front end
LB =  7.12593  # distance from rear to vehicle back end
MAX_STEER = 0.6  # [rad] maximum steering angle of the vehicle
print ("Vehicle Parameters:", "WB :", WB, "W: ",W, "LF: ",LF , "LB: ", LB )
#Create A buffer region (Circular) around the vehicle to check for collison avoidance
WBUBBLE_DIST = (LF - LB) / 2.0 
WBUBBLE_R = sqrt(((LF + LB) / 2.0)**2 + 1) #Radius to search points around the waypoints
#for collision avoidance 

# vehicle rectangle verticles; These would be the augmented points to consider 
# vehcile's dimension during plotting the harvey along the occupancy grid and waypoints
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W / 2, -W / 2, -W / 2, W / 2, W / 2]
```


base_global_planner_segment_
├─ .vscode
│  ├─ c_cpp_properties.json
│  └─ settings.json
├─ README.md
├─ config
│  ├─ DynamicParam.cfg
│  ├─ default.yaml
│  ├─ meerbusch_18_10_22.yaml
│  ├─ meerbusch_18_10_22_2.yaml
│  ├─ meerbusch_27_09_22.yaml
│  ├─ pose_offset.yaml
│  ├─ teb.yaml
│  └─ teb2.yaml
└─ scripts
   ├─ .ipynb_checkpoints
   ├─ __pycache__
   │  ├─ a_star_heuristic.cpython-37.pyc
   │  ├─ a_star_heuristic.cpython-38.pyc
   │  ├─ harvey.cpython-37.pyc
   │  ├─ harvey.cpython-38.pyc
   │  ├─ hybrid_a_star_user_input.cpython-37.pyc
   │  ├─ parameters.cpython-37.pyc
   │  ├─ parameters.cpython-38.pyc
   │  ├─ reeds_shepp_path.cpython-37.pyc
   │  └─ reeds_shepp_path.cpython-38.pyc
   ├─ a_star_heuristic.py
   ├─ car.py
   ├─ gpplanner_node.py
   ├─ gpplanner_node_circuit.py
   ├─ gpplanner_node_segment.py
   ├─ gpplanner_node_segment_automate_row.py
   ├─ gpplanner_node_segment_beta.py
   ├─ harvey.py
   ├─ hya*.ipynb
   ├─ hybrid_a_star_user_input.py
   ├─ parameters.py
   └─ reeds_shepp_path.py

