import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np
import math
from IPython import display

plt.xlabel("X-Axis")
plt.ylabel("Y-Axis")

def plot_path(path):
    # plots each point of the path
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], ".", color="royalblue", markersize=8)

    #plots lines between each point of the path
    for i in range(0, len(path)-1):
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color="royalblue")

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

    if (disc >= 0):
        intersect_found = True
        sol_x1 = (d * dy + sgn(dy) * dx * np.sqrt(disc)) / dr**2
        sol_x2 = (d * dy - sgn(dy) * dx * np.sqrt(disc)) / dr**2
        sol_y1 = (- d * dx + abs(dy) * np.sqrt(disc)) / dr**2
        sol_y2 = (- d * dx - abs(dy) * np.sqrt(disc)) / dr**2    
        
        sol1, sol2 = [sol_x1+current_x, sol_y1+current_y], [sol_x2+current_x, sol_y2+current_y]

    if (intersect_found == False):
        print("No intersection found")
    else:
        plt.plot([x1, x2], [y1, y2], "--", color="royalblue")
        draw_circle(current_pos, look_ahd_dist)
        print(f"Intersection found at {sol1} and {sol2}")
        highlight_points([sol1, sol2])
        plt.axis("scaled")
        plt.show()

line_circle_intersection([0, 1], [1, 1], [1, -4], 1)
    



    