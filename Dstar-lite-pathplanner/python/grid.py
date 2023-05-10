import numpy as np
from utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List
import math
import global_var

OBSTACLE = 255
WALL = 255
OBSTACLE_Z1 = 80
OBSTACLE_Z2 = 20
DYN_OBSTACLE = 100
UNOCCUPIED = 0

class OccupancyGridMap:

    def __init__(self, x_dim, y_dim, exploration_setting='8N'):
        """
        set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        :param x_dim: dimension in the x direction
        :param y_dim: dimension in the y direction
        """
        self.x_dim = x_dim
        self.y_dim = y_dim

        # the map extents in units [m]
        self.map_extents = (x_dim, y_dim)

        # the obstacle map
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)

#
#        # upper wall of the room
#        for i in range(5, y_dim-5):
#            self.occupancy_grid_map[5, i] = WALL
#            self.occupancy_grid_map[6, i] = OBSTACLE_Z1
#            self.occupancy_grid_map[7, i] = OBSTACLE_Z2

#            # bottom wall of the room
#        for i in range(5, y_dim-5):
#            self.occupancy_grid_map[x_dim - 6, i] = WALL
#            self.occupancy_grid_map[x_dim - 7, i] = OBSTACLE_Z1
#            self.occupancy_grid_map[x_dim - 8, i] = OBSTACLE_Z2

#            # left wall of the room
#        for i in range(5, x_dim-5):
#            self.occupancy_grid_map[i, 5] = WALL
#        for i in range(7, x_dim - 7):
#            self.occupancy_grid_map[i, 7] = OBSTACLE_Z2
#            self.occupancy_grid_map[i, 6] = OBSTACLE_Z1

#            # right wall of the room
#        for i in range(5,x_dim-5):
#            self.occupancy_grid_map[i, y_dim - 6] = WALL
#        for i in range(7, x_dim - 7):
#            self.occupancy_grid_map[i, y_dim - 8] = OBSTACLE_Z2
#            self.occupancy_grid_map[i, y_dim - 7] = OBSTACLE_Z1

#            # middle left wall of the room
#        for i in range(5, int((y_dim - 1)/2) - 5):
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 5, i] = WALL
#        for i in range(7, int((y_dim - 1) / 2) - 5):
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 3, i + 1] = OBSTACLE_Z2
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 4, i + 1] = OBSTACLE_Z1
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 7, i + 1] = OBSTACLE_Z2
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 6, i + 1] = OBSTACLE_Z1

#            # middle right wall of the room
#        for i in range(int((y_dim - 1)/2) + 5, y_dim - 5):
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 5, i] = WALL
#        for i in range(int((y_dim - 1) / 2) + 5, y_dim - 7):
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 3, i - 1] = OBSTACLE_Z2
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 4, i - 1] = OBSTACLE_Z1
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 7, i - 1] = OBSTACLE_Z2
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 6, i - 1] = OBSTACLE_Z1

#            # Some obstacles
#        for i in range(41, 52):
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 6, i] = OBSTACLE
#            self.occupancy_grid_map[int((x_dim - 1)/2) + 7, i] = OBSTACLE

#        for i in range(15, 27):
#            self.occupancy_grid_map[33, i] = OBSTACLE
#            self.occupancy_grid_map[34, i] = OBSTACLE

            # middle left wall of the room
        for i in range(0, y_dim - 35):
            self.occupancy_grid_map[x_dim - 13, i] = WALL
        for i in range(0, y_dim - 34):
            self.occupancy_grid_map[x_dim - 11, i] = OBSTACLE_Z2
            self.occupancy_grid_map[x_dim - 12, i] = OBSTACLE_Z1
            self.occupancy_grid_map[x_dim - 14, i] = OBSTACLE_Z1
            self.occupancy_grid_map[x_dim - 15, i] = OBSTACLE_Z2

            # middle right wall of the room
        for i in range(y_dim - 25, y_dim):
            self.occupancy_grid_map[x_dim - 13, i] = WALL
        for i in range(y_dim - 26, y_dim):
            self.occupancy_grid_map[x_dim - 11, i] = OBSTACLE_Z2
            self.occupancy_grid_map[x_dim - 12, i] = OBSTACLE_Z1
            self.occupancy_grid_map[x_dim - 15, i] = OBSTACLE_Z2
            self.occupancy_grid_map[x_dim - 14, i] = OBSTACLE_Z1

        # upper wall of the room
        for i in range(10, y_dim - 10):
            self.occupancy_grid_map[5, i] = WALL
            self.occupancy_grid_map[6, i] = OBSTACLE_Z1
            self.occupancy_grid_map[7, i] = OBSTACLE_Z2

            # bottom wall of the room
        for i in range(0, y_dim):
            self.occupancy_grid_map[x_dim - 5, i] = WALL
            self.occupancy_grid_map[x_dim - 6, i] = OBSTACLE_Z1
            self.occupancy_grid_map[x_dim - 7, i] = OBSTACLE_Z2

            # left wall of the room
        for i in range(5, x_dim - 13):
            self.occupancy_grid_map[i, 10] = WALL
        for i in range(7, x_dim - 15):
            self.occupancy_grid_map[i, 11] = OBSTACLE_Z1
            self.occupancy_grid_map[i, 12] = OBSTACLE_Z2

            # right wall of the room
        for i in range(5, x_dim - 13):
            self.occupancy_grid_map[i, y_dim - 10] = WALL
        for i in range(7, x_dim - 15):
            self.occupancy_grid_map[i, y_dim - 11] = OBSTACLE_Z1
            self.occupancy_grid_map[i, y_dim - 12] = OBSTACLE_Z2

        # Some obstacles
        for i in range(42, 51):
            self.occupancy_grid_map[x_dim - 14, i] = OBSTACLE
            self.occupancy_grid_map[x_dim - 15, i] = OBSTACLE
            # Ora of lower right obstacle (Noam)
            self.occupancy_grid_map[x_dim - 16, i - 1] = OBSTACLE_Z1
            self.occupancy_grid_map[x_dim - 17, i - 1] = OBSTACLE_Z2
        self.occupancy_grid_map[x_dim - 14, 41] = OBSTACLE_Z1
        self.occupancy_grid_map[x_dim - 15, 41] = OBSTACLE_Z1
        self.occupancy_grid_map[x_dim - 14, 40] = OBSTACLE_Z2
        self.occupancy_grid_map[x_dim - 15, 40] = OBSTACLE_Z2

        for i in range(14, 28):
            self.occupancy_grid_map[7, i] = OBSTACLE
            self.occupancy_grid_map[6, i] = OBSTACLE
            # Ora of upper left obstacle (Noam)
            self.occupancy_grid_map[8, i] = OBSTACLE_Z1
            self.occupancy_grid_map[9, i] = OBSTACLE_Z2
        self.occupancy_grid_map[6, 14] = OBSTACLE_Z1
        self.occupancy_grid_map[7, 14] = OBSTACLE_Z1
        self.occupancy_grid_map[6, 13] = OBSTACLE_Z2
        self.occupancy_grid_map[7, 13] = OBSTACLE_Z2
        self.occupancy_grid_map[6, 27] = OBSTACLE_Z1
        self.occupancy_grid_map[7, 27] = OBSTACLE_Z1
        self.occupancy_grid_map[6, 28] = OBSTACLE_Z2
        self.occupancy_grid_map[7, 28] = OBSTACLE_Z2

        # obstacles
        self.visited = {}
        self.exploration_setting = exploration_setting



    def get_map(self):
        """
        :return: return the current occupancy grid map
        """
        return self.occupancy_grid_map

    def set_map(self, new_ogrid):
        """
        :param new_ogrid:
        :return: None
        """
        self.occupancy_grid_map = new_ogrid

    # Checks if there is any obstacle/wall in this position
    def is_unoccupied(self, pos: (int, int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        # if not self.in_bounds(cell=(x, y)):
        #    raise IndexError("Map index out of bounds")
        is_unoccupied = True
        #if self.occupancy_grid_map[row][col] != UNOCCUPIED and self.occupancy_grid_map[row][col] != OBSTACLE_Z2:
        #    is_unoccupied = False
        #print(pos, is_unoccupied)
        #return is_unoccupied
        return self.occupancy_grid_map[row][col] == UNOCCUPIED

    def is_obstacles(self, pos: (int, int)) -> bool:
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        is_unoccupied = True
        if self.occupancy_grid_map[row][col] == OBSTACLE or self.occupancy_grid_map[row][col] == DYN_OBSTACLE or self.occupancy_grid_map[row][col] == WALL:
            is_unoccupied = False
        return is_unoccupied

    # Checks if there is a statis obstacle in this position (Dana)
    def is_static_obs(self, pos: (int, int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        is_unoccupied = True
        if self.occupancy_grid_map[row][col] == OBSTACLE:
            is_unoccupied = False
        return is_unoccupied

    # Checks if there is a dynamic obstacle in this position (Dana)
    def is_dyn_obs(self, pos: (int, int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        is_unoccupied = True
        if self.occupancy_grid_map[row][col] == DYN_OBSTACLE:
            is_unoccupied = False
        return is_unoccupied


    def in_bounds(self, cell: (int, int)) -> bool:
        """
        Checks if the provided coordinates are within
        the bounds of the grid map
        :param cell: cell position (x,y)
        :return: True if within bounds, False else
        """
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def filter(self, neighbors: List, avoid_obstacles: bool):
        """
        :param neighbors: list of potential neighbors before filtering
        :param avoid_obstacles: if True, filter out obstacle cells in the list
        :return:
        """
        if avoid_obstacles:
            return [node for node in neighbors if self.in_bounds(node) and self.is_unoccupied(node)]
        return [node for node in neighbors if self.in_bounds(node)]

    # This function checks all the points around the robot/dynamic obstacle position to calculate the next step (Dana)
    def succ(self, vertex: (int, int), avoid_obstacles: bool = False) -> list:
        """
        :param avoid_obstacles:
        :param vertex: vertex you want to find direct successors from
        :return:
        """
        (x, y) = vertex

        if self.exploration_setting == '4N':  # change this
            movements = get_movements_4n(x=x, y=y)
        else:
            movements = get_movements_8n(x=x, y=y)
        # not needed. Just makes aesthetics to the path
        if (x + y) % 2 == 0:
            movements.reverse()

        filtered_movements = self.filter(neighbors=movements, avoid_obstacles=avoid_obstacles)
        return list(filtered_movements)

    def set_obstacle(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = OBSTACLE

    def set_dynamic_obstacle(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = DYN_OBSTACLE
    def set_dynamic_obstacle_neighbors(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = OBSTACLE_Z2
    def remove_obstacle(self, pos: (int, int)):
        """
        :param pos: position of obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = UNOCCUPIED



    # Find the free locations for setting the obstacles start position and goal (Dana)
    def find_zeros(self):
        zeros = []
        for x in range(5, self.x_dim-13):
            for y in range(10, self.y_dim-10):
                pos = (x, y)
                if self.is_unoccupied(pos=pos) and pos != (15, 15) and pos != (32, 19) and pos != (17, 45):
                    zeros.append((x, y))
        return zeros


    # Find the free locations for setting the obstacles start position and goal (Dana)
    def find_zeros_in_hall_left(self):
        zeros = []
        for x in range(self.x_dim-10, self.x_dim-7):
            print(x)
            for y in range(1, 5):
                pos = (x, y)
                if self.is_unoccupied(pos=pos) and pos != (15, 15) and pos != (32, 19) and pos != (17, 45):
                    zeros.append((x, y))
        return zeros

    def find_zeros_in_hall_right(self):
        zeros = []
        for x in range(self.x_dim-10, self.x_dim-7):
            for y in range(self.y_dim - 5, self.y_dim-1):
                pos = (x, y)
                if self.is_unoccupied(pos=pos) and pos != (15, 15) and pos != (32, 19) and pos != (17, 45):
                    zeros.append((x, y))
        return zeros
    
    def local_observation(self, global_position: (int, int), view_range: int = 3) -> Dict:
        (px, py) = global_position
        # saves all the coordinates in the circle range
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}

    def dyn_obs_neighbors(self, global_position: (int, int)) -> Dict:
        (px, py) = global_position
        # saves all the coordinates in the circle range
        nodes = [(x, y) for x in range(px - 2, px + 2)
                 for y in range(py - 2, py + 2)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= 1]
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}


    # Count static obstacles (Dana)
    def count_obstacles_local_observation(self, global_position: (int, int), view_range: int = 3):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        count = 0
        arr = []
        for node in nodes:
            if self.occupancy_grid_map[node] == OBSTACLE:
                arr.append(node)
                count += 1
        return count

    # Count dynamic obstacles (Dana)
    def count_dynamic_obstacles_local_observation(self, global_position: (int, int), view_range: int = 3):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        count = 0
        for node in nodes:
            if self.occupancy_grid_map[node] == DYN_OBSTACLE:
                count += 1
        return count

    # Checks minimal distance between obstacles (Dana)
    def minimal_distance_local_observation(self, global_position: (int, int), view_range: int = 3):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        min_dist = 7.0
        for node in nodes:
            if not self.is_obstacles(pos=node):
                dist = math.dist(node, global_position)
                if min_dist > dist:
                    min_dist = dist
        if min_dist >= 7.0:
            return 0
        return min_dist

    def average_distance_local_observation(self, global_position: (int, int), view_range: int = 3):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        total_distance = 0
        obstacles = []
        for node in nodes:
            if not self.is_obstacles(pos=node):  # means that it is an obstacle
                obstacles.append(node)
        for i in range(len(obstacles)):
            for j in range(i + 1, len(obstacles)):
                # calculate the distance between the current pair of obstacles
                distance = math.dist(obstacles[i], obstacles[j])
                total_distance += distance
        num_obstacles = len(obstacles)
        if num_obstacles > 1:
            average_distance = total_distance / (num_obstacles * (num_obstacles - 1) / 2)
            return average_distance
        return 0

    def largest_angle_local_observation(self, global_position: (int, int), view_range: int = 5):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        obstacles = []
        for node in nodes:
            if not self.is_obstacles(pos=node):  # means that it is an obstacle
                obstacles.append(node)
        largest_angle = 0
        angles = []
        for x, y in obstacles:
            # calculate the angle between the center of the circle and the obstacle in radians
            angle = math.atan2(y - py, x - px)
            # convert the angle from radians to degrees
            angle = math.degrees(angle)
            # add the angle to the list
            angles.append(angle)
        # sort the angles in ascending order
        angles.sort()
        # initialize the largest free angle
        largest_free_angle = 0
        # iterate through the sorted angles
        for i in range(len(angles) - 1):
            # calculate the difference between the current angle and the next one
            angle_diff = angles[i + 1] - angles[i]
            # update the largest free angle if necessary
            largest_free_angle = max(largest_free_angle, angle_diff)
        if len(obstacles) == 2:
            return (360 - largest_free_angle)
        return largest_free_angle

    # Checks the current goal of the robot (Dana)
    def section_of_fail(self):
        if global_var.arrivedA == True and global_var.arrivedB1 == False and global_var.arrivedB2 == False and \
                global_var.arrivedC == False:
            return "A to B"
        if global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == True \
                and global_var.arrivedC == True:
            return "B to A"
        if global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False\
                and global_var.arrivedC == False:
            return "B to C"
        if global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == True:
            return "C to B"
        else:
            return "Null"

class SLAM:
    def __init__(self, map: OccupancyGridMap, view_range: int):
        self.ground_truth_map = map
        self.slam_map = OccupancyGridMap(x_dim=map.x_dim,
                                         y_dim=map.y_dim)
        self.view_range = view_range
        self.vector = []

    def set_ground_truth_map(self, gt_map: OccupancyGridMap):
        self.ground_truth_map = gt_map

    def c(self, u: (int, int), v: (int, int)) -> float:
        """
        calcuclate the cost between nodes
        :param u: from vertex
        :param v: to vertex
        :return: euclidean distance to traverse. inf if obstacle in path
        """
        if not self.slam_map.is_unoccupied(u) or not self.slam_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def dyn_obs_neighbors(self, global_position: (int, int), view_range: int = 1) -> Dict:
        (px, py) = global_position
        # saves all the coordinates in the circle range
        nodes =[(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.ground_truth_map.in_bounds((x, y)) and math.dist((x, y), global_position) <= math.sqrt(2)]
        return {node: UNOCCUPIED if self.ground_truth_map.is_unoccupied(pos=node) else OBSTACLE_Z1 for node in nodes}

    def rescan(self, global_position: (int, int)):
        if self.vector==[]:
            s = ('Number of static obstacles', 'Number of dynamic obstacles', 'Minimum distance from obstacle', 'Average distance from obstacles', 'Largest free angle', 'Section of fail', 'Run number', 'Robot position')
            self.vector.append(s)

        # rescan local area
        local_observation = self.ground_truth_map.local_observation(global_position=global_position,
                                                                    view_range=self.view_range)

        num_obstacles = self.ground_truth_map.count_obstacles_local_observation(global_position=global_position,
                                                                                view_range=self.view_range)

        num_dynamic_obstacles = self.ground_truth_map.count_dynamic_obstacles_local_observation(global_position=global_position,
                                                                               view_range=self.view_range)

        min_distance = self.ground_truth_map.minimal_distance_local_observation(global_position=global_position,
                                                                                view_range=self.view_range)

        average_distance = self.ground_truth_map.average_distance_local_observation(global_position=global_position,
                                                                                    view_range=self.view_range)

        largest_angle = self.ground_truth_map.largest_angle_local_observation(global_position=global_position,
                                                                              view_range=self.view_range)

        section_of_fail = self.ground_truth_map.section_of_fail()
        status = (num_obstacles, num_dynamic_obstacles, min_distance, average_distance, largest_angle, section_of_fail, global_var.counter_runs, global_position)
        self.vector.append(status)
        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map

    def update_changed_edge_costs(self, local_grid: Dict) -> Vertices:
        vertices = Vertices() # returns all the neighbors
        for node, value in local_grid.items():
            # if obstacle
            if value == OBSTACLE: # everything counts as an obstacle (Dana)
                if self.slam_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)
                    self.slam_map.set_obstacle(node)

            else:
                # if white cell
                if not self.slam_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)
                    self.slam_map.remove_obstacle(node)
        return vertices
