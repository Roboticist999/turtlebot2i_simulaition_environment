#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import pow


#check to see any neighbors of a point are free space
def check_neighbors_for_free_space(occu_grid,row,col):

    if row == 0:
        top_left = False
        top = False
        top_right = False
    if col == 0:
        top_left = False
        left = False
        bottom_left = False
    if row == len(occu_grid):
        bottom_left = False
        bottom = False
        bottom_right = False

    if top_left is None:
        top_left = (occu_grid[row-1][col-1] == 0)
    if top is None:
        top = (occu_grid[row-1][col] == 0)
    if top_right is None:
        top_right = (occu_grid[row-1][col+1] == 0)
    if left is None:
        left = (occu_grid[row][col-1] == 0)
    if right is None:
        right = (occu_grid[row][col+1] == 0)
    if bottom_left is None:
        bottom_left = (occu_grid[row+1][col-1] == 0)
    if bottom is None:
        bottom = (occu_grid[row+1][col] == 0)
    if bottom_right is None:
        bottom_right = (occu_grid[row+1][col+1] == 0)

    top_row = top_left and top and top_right
    mid = left and right
    bottom_row = bottom_left and bottom and bottom_right

    return (top_row and mid and bottom_row)

def near_point(f,point,threshold):

    for f_data in f:
        if abs(f_data[0] - point[0]) =< threshold:
            return True
        if abs(f_data[1] - point[1]) =< threshold:
            return True
    return False

#occupancy grid is formatted:
#-1 is unkown
#0 is free space
#100 is known obstacle
def find_frontiers(occu_grid):
    #get grid size:
    [height, width] = [len(occu_grid),len(occu_grid[0])]
    #initialize frontier list
    frontiers = []

    #iterate through all points, adding them to relevant 
    for row in height:
        for col in width:
            #check if data point is unknown and has free_space neighhbors
            #means check if considered frontier
            if occu_grid[row][col] == -100 and check_neighbors_for_free_space(occu_grid,row,col):
                #find correct existing frontier to add point to or add new frontier
                added = False
                for f in frontiers:
                    if near_point(f,data_point,1):
                        f.append([row,col])
                        added = True
                if not added:
                    frontiers.append([[row,col]])

    return frontiers

#subscribes to map data, then removes itself once it has it
def map_callback(msg):
    global got_map
    global map_data
    got_map = True
    global last_got_map = rospy.Time.now()
    map_data = msg
    if got_map:
        map_sub.unregister()

#subscribe to map
def get_map():
    if got_map is not None and got_map:
        return map_data
    else:
        got_map = False
        start_get_map = rospy.Time.now()
        map_sub = rospy.Subscriber('map', OccupancyGrid, map_callback)
        #wait until you get the map or 5 seconds
        while not got_map and (rospy.Time.now() > start_get_map + 5*1000):
            rospy.spin()
        if got_map:
            return map_data
        else:
            print('could not find map')
            return Exception

#subscribes to map data, then removes itself once it has it
def pose_callback(msg):
    global got_pose
    global pose_data
    got_pose = True
    global last_got_pose = rospy.Time.now()
    pose_data = msg
    if got_pose:
        pose_sub.unregister()

#subscribe to pose
def get_pose():
    if got_pose is not None and got_pose:
        return pose_data
    else:
        got_pose = False
        start_get_pose = rospy.Time.now()
        pose_sub = rospy.Subscriber('pose', OccupancyGrid, pose_callback)
        #wait until you get the map or 5 seconds
        while not got_pose and (rospy.Time.now() > start_get_pose + 5*1000):
            rospy.spin()
        if got_pose:
            return map_pose
        else:
            print('could not find pose')
            return Exception

rospy.init_node('explore')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(2)

#get initial map and frontiers
get_map()
get_pose()
searchable_frontiers = find_frontiers(map_data)

def xy_dist_from_veh(point):
    x_dist = veh_pose.position.x - point[0]
    y_dist = veh_pose.position.y - point[1]

    return sqrt(pow(x_dist,2) + pow(y_dist,2))
    
goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

while not rospy.is_shutdown() and (searchable_frontiers > 0):

    #pick largest frontier to work with for maximum info gain
    #note, this approach is not consdiering current robot position when deciding on ideal search location
    largest_frontier = []
    current_frontier = 0
    searchable_frontiers = searchable_frontiers.sort(key=len)
    
    
    #find median location of  of current frontier to get navigation goal
    frontier_to_use = searchable_frontiers[0]
    veh_pose = get_pose()
    frontier_to_use.sort(key=xy_dist_from_veh)
    #pick median location
    goal_diff = frontier_to_use[len(frontier_to_use)/2 + 1]

    goal_pose = veh_pose
    goal_pose.position.x = goal_pose.position.x + goal_diff[0]
    goal_pose.position.y = goal_pose.position.y + goal_diff[1]

    print('looking for map data')
    print(rospy.Time.now())
    rate.sleep()



