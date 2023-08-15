import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np
import math

plt.xlabel("X-Axis")
plt.ylabel("Y-Axis")

#plots the path on the grid
def plot_path(path, color):
    # plots each point of the path
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], ".", color=color, markersize=8)

    #plots lines between each point of the path
    for i in range(0, len(path)-1):
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color=color)

#plots the current heading of the robot
def plot_theta():
    global current_pos, theta, look_ahd_dist
    x, y, = current_pos[0], current_pos[1]
    endy = y + look_ahd_dist * math.sin(theta)
    endx = x + look_ahd_dist * math.cos(theta)

    plt.plot([x, endx], [y, endy], color="red", markersize = 5)
#changes the designated points to red
def highlight_points(points):
    for point in points :
        plt.plot(point[0], point[1], '.', color="red", markersize = 10)

#draw a circle at (x, y) with specified radius
def draw_circle(xy, radius):
    circle = plt.Circle(xy, radius, fill=False)
    plt.gca().add_patch(circle)

#determines the sign of a number
def sgn(num):
    if (num < 0):
        return -1
    else:
        return 1
    
#returns the distance between two points
def get_dist(pt1, pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

#returns 0, 1, or 2 valid intersections
def line_circle_intersection(current_pos, pt1, pt2, look_ahd_dist):
    x1, x2, y1, y2 = pt1[0], pt2[0], pt1[1], pt2[1]
    current_x, current_y = current_pos[0], current_pos[1]
    intersect_found = False
    print(f"Currently searching between pt {x1, y1} and pt {x2, y2}")

    #offset to the origin
    x1_offset = x1 - current_x
    x2_offset = x2 - current_x
    y1_offset = y1 - current_y
    y2_offset = y2 - current_y

    #use discriminant to determine if there are any intersections
    dx = x2_offset - x1_offset
    dy = y2_offset - y1_offset
    dr = math.sqrt((dx**2) + (dy**2))
    d = (x1_offset*y2_offset) - (x2_offset*y1_offset)
    disc = (look_ahd_dist**2) * (dr**2) - d**2

    #use quadratic formula to find intersection(s)
    if (disc >= 0):
        intersect_found = True
        sol_x1 = (d * dy + sgn(dy) * dx * np.sqrt(disc)) / dr**2
        sol_x2 = (d * dy - sgn(dy) * dx * np.sqrt(disc)) / dr**2
        sol_y1 = (- d * dx + abs(dy) * np.sqrt(disc)) / dr**2
        sol_y2 = (- d * dx - abs(dy) * np.sqrt(disc)) / dr**2    
        #reset the system
        sol1, sol2 = [sol_x1+current_x, sol_y1+current_y], [sol_x2+current_x, sol_y2+current_y]
    
    if (intersect_found == False):
        return False
    else:
        validpts = []
        #check if the intersections are in between the points
        minX = min(x1, x2)
        maxX = max(x1, x2)
        if (minX <= sol1[0] <= maxX):
            validpts.append(sol1)
        if (minX <= sol2[0] <= maxX):
            validpts.append(sol2)
        return validpts

        #plot example
        # plt.plot([x1, x2], [y1, y2], "--", color="royalblue")
        # draw_circle(current_pos, look_ahd_dist)
        # print(f"Intersection found at {sol1} and {sol2}")
        # highlight_points([sol1, sol2])
        # plt.axis("scaled")
        # plt.show()
  
#sets the goal pt based on valid intersections
def goal_pt_search():
    global path, look_ahd_dist, last_found_index, current_pos

    for i in range(last_found_index, len(path)):
        valid_pts = line_circle_intersection(current_pos, path[i], path[i+1], look_ahd_dist)
        #no intersections were found, robot has strayed off the path
        if valid_pts == False:
            return path[last_found_index]
        #intersections were found but no valid points, move on to next iteration of the loop
        if not valid_pts:
            print("No Valid Points")
            continue
        #if there is only one valid point, take that one
        if (len(valid_pts) == 1):
            sol_pt = valid_pts[0]
        #if there are two valid points, choose the one that is closest to the end of the path
        else:
            if (get_dist(valid_pts[0], path[i+1]) < get_dist(valid_pts[1], path[i+1])):
                sol_pt = valid_pts[0]
            else:
                sol_pt = valid_pts[1]

        #check if the goal pt is closer to the end than the current position of the robot
        if (get_dist(sol_pt, path[i+1]) < get_dist(current_pos, path[i+1])):
            last_found_index = i
            goal_pt = sol_pt
            return goal_pt
            
            # print(goal_pt)
            # plt.plot(goal_pt[0], goal_pt[1], ".", color="red", markersize = 15)
            # plot_path(path[0:last_found_index+1], "brown")
            # plot_path(path[last_found_index:], "grey")
            # draw_circle(current_pos, look_ahd_dist)
            # plt.show()                

                
        #plot the example
        # plot_path(path[0:last_found_index+1], "brown")
        # plot_path(path[last_found_index:], "grey")
        # draw_circle(current_pos, look_ahd_dist)
        # plt.show()

#returns the angle of the goalpt relative to the current position of the robot
def get_goal_angle():
    goal_offsetx = goal_pt[0] - current_pos[0]
    goal_offsety = goal_pt[1] - current_pos[1]
    return math.atan2(goal_offsety, goal_offsetx)

#returns the turn error
def get_steering_angle():
    #subtract the current angle of the robot from the angle of the goal point
    global theta
    #offset current position to the origin
    goal_angle = get_goal_angle()
    print(f"Goal Angle: {goal_angle}")
    print(f"Current Angle: {theta}")
    steering_angle = goal_angle - theta
    print(f"Steering Angle: {steering_angle}")
    return steering_angle

#plots the path, current position, goalpt, theta, and traveled path
def update():
    plot_path(path, "grey")
    plot_theta()
    plt.plot(current_pos[0], current_pos[1], ".", color="red", markersize=15)
    plt.plot(robot_pathx, robot_pathy, color="brown", markersize=.5)
    plt.plot(goal_pt[0], goal_pt[1], "x", color="red", markersize = 15)
    draw_circle(current_pos, look_ahd_dist)
    plt.axis("equal")    
    plt.show()    

#Robot Path
path = [[0.0, 0.0], [0.08, 0.4], [0.18, 1.3], [0.4, 1.68], [0.94, 1.86], [1.62, 2.01], [2.32, 1.98], [3, 1.87], [3.58, 1.62], [3.92, 1.37], [3.89, 0.74], [3.62, 0.26], [3.12, 0.15], [2.4, 0.052], [1.47, 0.003], [0.64, 0.0012], [0, 0]]

#Parameters
current_pos = [0,0]
theta = 0
look_ahd_dist = .5
last_found_index = 0
goal_pt = []

wheel_radius = .1
wheelbase = .5
linear_vel = .2
dt = .2 #time step

robot_pathx = []
robot_pathy = []

#algorithm
while True:
    #get the goal point and update last_found_index
    goal_pt = goal_pt_search()
    #calculations
    steering_angle = get_steering_angle()
    if (steering_angle > (3*math.pi)/2):
        steering_angle -= (2*math.pi)
    # if the steering angle is too large, make the robot turn in place until the steering angle is small enough
    if (math.pi/2 < abs(steering_angle) < (3*math.pi)/2):
        theta = get_goal_angle()
        steering_angle = get_steering_angle()
    linear_error = get_dist(current_pos, goal_pt)
    if (steering_angle == 0):
        lwheel_speed = linear_vel
        rwheel_speed = linear_vel
    else:
        turning_radius = linear_error/(2*math.sin(steering_angle))
        # print(f"Turning Radius: {turning_radius}\n")
        
        if (abs(turning_radius) > (wheelbase/2)):
            lwheel_speed = abs((linear_vel/wheel_radius)*(turning_radius-(wheelbase/2)))
            rwheel_speed = abs((linear_vel/wheel_radius)*(turning_radius+(wheelbase/2)))
        else:
            lwheel_speed = abs((linear_vel/wheel_radius)*((wheelbase/2)-turning_radius))
            rwheel_speed = abs((linear_vel/wheel_radius)*((wheelbase/2)+(turning_radius)))

    #plot everything before the position is updated
    # print(f"Left Wheel Speed:{lwheel_speed}; Right Wheel Speed: {rwheel_speed}")
    update()
    omega = (wheel_radius/wheelbase) * (rwheel_speed - lwheel_speed)
    theta += omega * dt
    current_pos[0] += linear_vel * math.cos(theta) * dt
    current_pos[1] += linear_vel * math.sin(theta) * dt
    robot_pathx.append(current_pos[0])
    robot_pathy.append(current_pos[1])
    # print(f"Omega: {omega}")
    

