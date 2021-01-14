#!/usr/bin/env python
"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""


import math
import numpy as np
import matplotlib.pyplot as plt
from baziercurve import bezier_curve

show_animation = False


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)
        self.BinaryObstacle_Map = None

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(int(self.y_width))]
                             for j in range(int(self.x_width))]
        for ix in range(int(self.x_width)):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(int(self.y_width)):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

        self.BinaryObstacle_Map = np.zeros(np.shape(self.obstacle_map))
        for i in range(0, np.shape(self.obstacle_map)[0]):
            for j in range(0,np.shape(self.obstacle_map)[1]):
                if self.obstacle_map[i][j] == True:
                    self.BinaryObstacle_Map[i,j] = 1
                else:
                    self.BinaryObstacle_Map[i,j] = 0

        #print(self.BinaryObstacle_Map)
        #print(self.obstacle_map)
        #print(np.shape(self.BinaryObstacle_Map))
        #print(np.shape(self.obstacle_map))

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


#############################################
## Extra Functions fro Smooting our Path
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):
    pruned_path = [p for p in path]
    pruned_path = path
    print('in prune path')
    #print(pruned_path)

    i = 0
    while i < (len(pruned_path) - 2):
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])

        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path

#############################################

def PathPlanning():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10  # [m]
    sy = 10  # [m]
    gx = 125 # [m]
    gy = 300  # [m]
    grid_size =10  # [m]
    robot_radius = 10  # [m]

    # set obstacle positions
    ox, oy = [], []
    # Bottom Wall
    for i in range(0, 150):
        ox.append(i)
        oy.append(0.0)
    # Left Wall
    for i in range(0, 340):
        ox.append(0.0)
        oy.append(i)
    # Top Wall
    for i in range(0,150):
        ox.append(i)
        oy.append(340)
    # Right Wall
    for i in range(0, 340):
        ox.append(150)
        oy.append(i)

    wall_thickness = 10
    wall_hole_width = 70
    # tagCoordinate = np.array([msg.pose.pose.pose.position.x, msg.pose.pose.pose.position.y])
    tagCoordinate = [50, 130]
    for i in range(0, 150):
        for j in range(tagCoordinate[1], tagCoordinate[1] + wall_thickness):
            if abs(i - tagCoordinate[0] - 0.5*wall_hole_width ) >= wall_hole_width*0.5: 
                ox.append(i)
                oy.append(j)


    box = 20  # square box
    radius = math.sqrt(2*box**2)

    boxCoordinate = np.array([100, 250])
    # print(boxCoordinate[1])
    for i in range(0,150):
        for j in range(0, 340):
            if (i-boxCoordinate[0])**2 + (j-boxCoordinate[1])**2 <= radius**2:
                ox.append(i)
                oy.append(j)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")


    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    #print("rx",rx)
    #print("ry",ry)
    path_2d = np.zeros((len(rx),2))
    for i in range(len(rx)):
        path_2d[i][0] = rx[i]
        path_2d[i][1] = ry[i]
    path_2d = path_2d[::-1]
    #print(path_2d)

    xvals, yvals = bezier_curve(path_2d, nTimes=50)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.plot(xvals, yvals, 'o')
        #plt.plot(xvals, yvals, 'o', t, p(t), '-')
        plt.pause(0.001)
        plt.show()



if __name__ == '__main__':
    PathPlanning()
