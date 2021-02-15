#!/usr/bin/env python
from enum import Enum
from queue import PriorityQueue
import numpy as np
import math
from bresenham import bresenham

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    # TODO: DIMENSION OF OUR GAZEBO WORLD
    # north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    # north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    north_min = 0
    north_max = 340
    print(north_min,north_max)

    # minimum and maximum east coordinates
    # east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    # east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    east_min = 0
    east_max = 150
    print(east_min,east_max)

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))
    print(north_size,east_size)


    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1


    return grid, int(north_min), int(east_min)

def createGazeboGrid():
    north_min = 0
    north_max = 34
    east_min = 0
    east_max = 15

    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))
    print(north_size,east_size)

    # Initialize an empty grid
    grid = np.zeros((east_size,north_size))
    print('size of grid')
    print(np.shape(grid))
    grid_2 = np.zeros((5,10))
    for i in range(0,np.shape(grid)[0]):
        grid[i,0] =1
        grid[i,-1]= 1
    for j in range(0,np.shape(grid)[1]):
        grid[0,j] = 1
        grid[-1,j]= 1
    print(grid[0:10,0:10])

    return grid



# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    
    NORTH_WEST = (-1, -1, math.sqrt(2))
    NORTH_EAST = (-1, 1, math.sqrt(2))
    SOUTH_WEST = (1, -1, math.sqrt(2))
    SOUTH_EAST = (1, 1, math.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        
    if (x-1 < 0) or (y-1 < 0) or grid[x-1, y-1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x-1 < 0) or (y+1 > m) or grid[x-1, y+1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x+1 > n) or (y+1 > m) or grid[x+1, y+1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if (x+1 > n) or (y-1 < 0) or grid[x+1, y-1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)

    return valid_actions


def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, grid, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path, grid):
    pruned_path = [p for p in path]
    
    i = 0
    while i < (len(pruned_path) - 2):
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        if collinearity_check(p1, p2, p3, grid):
            pruned_path.remove(pruned_path[i+1])
        else:
            i +=1
    return pruned_path


if __name__ == "__main__":

    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5  # safety distance from obstacles
    grid = createGazeboGrid()

    # Read in obstacle map TODO: WE HAVE TO CONVERT OUR OBSTACLES SOME HOW INTO CSV File *****
    #data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
    # grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    # print(np.shape(grid))
    # print(grid[0:,0:20])
    # # Starting poistion
    # current_local_pos = global_to_local(self.global_position, self.global_home)
    # grid_start = (int(current_local_pos[0] - north_offset), int(current_local_pos[1] - east_offset))
    #
    #
    # # Take GPS co-ordinates as Grid goal -Me
    # grid_goal = (-122.396582, 37.795714, 0)
    # grid_goal = global_to_local(grid_goal, self.global_home)
    # grid_goal = (int(grid_goal[0] - north_offset), int(grid_goal[1] - east_offset))
    #
    #
    # # Run A* to find a path from start to goal
    # # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
    # # or move to a different search space such as a graph (not done here)
    # print('Local Start and Goal: ', grid_start, grid_goal)
    # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
    #
    # # TODO: prune path to minimize number of waypoints
    # # TODO (if you're feeling ambitious): Try a different approach altogether!
    # path = prune_path(path, grid)
    #
    # # Convert path to waypoints
    # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
    # # Set self.waypoints
