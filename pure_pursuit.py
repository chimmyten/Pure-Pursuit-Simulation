import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np
import math
from IPython import display

plt.xlabel("X-Axis")
plt.ylabel("Y-Axis")

def plot_path(path, color):
    # plots each point of the path
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], ".", color=color, markersize=8)

    #plots lines between each point of the path
    for i in range(0, len(path)-1):
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color=color)

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
    
def get_dist(pt1, pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt2[0])**2)

def line_circle_intersection(current_pos, pt1, pt2, look_ahd_dist):
    x1, x2, y1, y2 = pt1[0], pt2[0], pt1[1], pt2[1]
    current_x, current_y = current_pos[0], current_pos[1]
    intersect_found = False

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
        if (sol2[0] < minX or sol2[0] > maxX):
            validpts.append(sol2)
        return validpts

        #plot example
        # plt.plot([x1, x2], [y1, y2], "--", color="royalblue")
        # draw_circle(current_pos, look_ahd_dist)
        # print(f"Intersection found at {sol1} and {sol2}")
        # highlight_points([sol1, sol2])
        # plt.axis("scaled")
        # plt.show()
  
def goal_pt_search(current_pos, path, last_found_index, look_ahd_dist):
    
    for i in range(last_found_index, len(path)):
        print(i)
        valid_pts = line_circle_intersection(current_pos, path[i], path[i+1], look_ahd_dist)
        #no intersections were found, robot has strayed off the path
        if valid_pts == False:
            goal_pt = path[last_found_index]
        #intersections were found but no valid points, move on to next iteration of the loop
        if not valid_pts:
            print("No Valid Points")
            continue
        #if there is only one valid point, take that one
        if (len(valid_pts) == 1):
            sol_pt = valid_pts
        #if there are two valid points, choose the one that is closest to the next point in the path
        else:
            if (get_dist(valid_pts[0], path[i+1]) < get_dist(valid_pts[1], path[i+1])):
                sol_pt = valid_pts[0]
            else:
                sol_pt = valid_pts[1]

            #check if the goal pt is closer to the end than the current position of the robot
            if (get_dist(sol_pt, path[i+1]) < get_dist(current_pos, path[i+1])):
                last_found_index = i
                goal_pt = sol_pt
                plt.plot(goal_pt[0], goal_pt[1], ".", color="red", markersize = 15)
                print(goal_pt)
                # plot_path(path[0:last_found_index+1], "brown")
                # plot_path(path[last_found_index:], "grey")
                # draw_circle(current_pos, look_ahd_dist)
                # plt.show()
                break                

                
        #plot the example
        # plot_path(path[0:last_found_index+1], "brown")
        # plot_path(path[last_found_index:], "grey")
        # draw_circle(current_pos, look_ahd_dist)
        # plt.show()

def get_steering_angle(current_angle):
    #subtract the current angle of the robot from the angle of the goal point
    goal_angle = math.atan2(goal_pt[1], goal_pt[0]) * (180/math.pi)
    steering_angle = goal_angle - current_angle
    return steering_angle


path = [[0.0, 0.0], [0.011580143395790051, 0.6570165243709267], [0.07307496243411533, 1.2724369146199181], [0.3136756819515748, 1.7385910188236868], [0.8813313906933087, 1.9320292911046681], [1.6153051608455251, 1.9849785681091774], [2.391094224224885, 1.9878393390954208], [3.12721333474683, 1.938831731115573], [3.685011039017028, 1.7396821576569221], [3.9068092597113266, 1.275245079016133], [3.9102406525571713, 0.7136897450501469], [3.68346383786099, 0.2590283720040381], [3.1181273273535957, 0.06751996250999465], [2.3832776875784316, 0.013841087641154892], [1.5971423891000605, 0.0023698980178599423], [0.7995795475309813, 0.0003490964043320208], [0, 0]]
current_pos = path[0]
current_angle = 0
look_ahd_dist = .8
last_found_index = 0
#algorithm

#while the end has not been reached
while (last_found_index < (len(path)-1)):
    #get the goal point and update last_found_index
    goal_pt = goal_pt_search(current_pos, path, last_found_index, look_ahd_dist)
    
    #calculations
    steering_angle = get_steering_angle(current_angle)
    linear_error = get_dist(current_pos, goal_pt)
    turning_radius = linear_error/(2*math.sin(steering_angle))
    