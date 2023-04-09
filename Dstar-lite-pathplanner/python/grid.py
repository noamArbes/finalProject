import numpy as np
from utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List
from Dynamic import dynamic_obs
import math
import random
import csv
import pygame
import global_var

OBSTACLE = 255
WALL = 200
OBSTACLE_Z1 = 80
OBSTACLE_Z2 = 20
DYN_OBSTACLE = 100
DYN_OBSTACLE_T2 = 150
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

        #d = self.set_dynamic_obstacle(20,20)
        # we can define here all the dynamic obstacles (Noam)
        # upper wall of the room
        for i in range(11, 94):
            self.occupancy_grid_map[7, i] = WALL
            self.occupancy_grid_map[8, i] = OBSTACLE_Z1
            self.occupancy_grid_map[9, i] = OBSTACLE_Z2

            # bottom wall of the room
        for i in range(10, 95):
            self.occupancy_grid_map[64, i] = WALL
            self.occupancy_grid_map[63, i] = OBSTACLE_Z1
            self.occupancy_grid_map[62, i] = OBSTACLE_Z2

            # left wall of the room
        for i in range(7, 65):
            self.occupancy_grid_map[i, 10] = WALL
        for i in range(8, 64):
            self.occupancy_grid_map[i, 11] = OBSTACLE_Z1
            self.occupancy_grid_map[i, 12] = OBSTACLE_Z2

            # right wall of the room
        for i in range(7, 65):
            self.occupancy_grid_map[i, 94] = WALL
        for i in range(8, 64):
            self.occupancy_grid_map[i, 93] = OBSTACLE_Z1
            self.occupancy_grid_map[i, 92] = OBSTACLE_Z2

            # middle left wall of the room
        for i in range(10, 35):
            self.occupancy_grid_map[43, i] = WALL
            self.occupancy_grid_map[42, i + 1] = OBSTACLE_Z1
            self.occupancy_grid_map[41, i + 1] = OBSTACLE_Z2
            self.occupancy_grid_map[44, i + 1] = OBSTACLE_Z1
            self.occupancy_grid_map[45, i + 1] = OBSTACLE_Z2

            # middle right wall of the room
        for i in range(70, 95):
            self.occupancy_grid_map[43, i] = WALL
            self.occupancy_grid_map[42, i - 1] = OBSTACLE_Z1
            self.occupancy_grid_map[41, i - 1] = OBSTACLE_Z2
            self.occupancy_grid_map[44, i - 1] = OBSTACLE_Z1
            self.occupancy_grid_map[45, i - 1] = OBSTACLE_Z2

        for i in range(75, 90):
            self.occupancy_grid_map[44, i] = WALL
            self.occupancy_grid_map[45, i] = WALL

        for i in range(75, 90):
            self.occupancy_grid_map[45, i] = WALL
            self.occupancy_grid_map[44, i] = WALL

        for i in range(75, 90):
            self.occupancy_grid_map[45, i] = WALL
            self.occupancy_grid_map[44, i] = WALL


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

    def is_unoccupied(self, pos: (int, int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        # if not self.in_bounds(cell=(x, y)):
        #    raise IndexError("Map index out of bounds")


        return self.occupancy_grid_map[row][col] == UNOCCUPIED

    def is_static_obs(self, pos: (int, int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        is_unoccupied = True
        if self.occupancy_grid_map[row][col] == 255 or self.occupancy_grid_map[row][col] == 100:
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

    def succ(self, vertex: (int, int), avoid_obstacles: bool = True) -> list:
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

    def remove_obstacle(self, pos: (int, int)):
        """
        :param pos: position of obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = UNOCCUPIED


    def local_observation(self, global_position: (int, int), view_range: int = 7) -> Dict:
        (px, py) = global_position
        # saves all the coordinates in the circle range
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        # we need check if we need to understand how to check if dynamic or static (Dana)
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}

    def count_obstacles_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        count = 0
        for node in nodes:
            if self.occupancy_grid_map[node] == OBSTACLE:
                count += 1
        return count

    def count_dynamic_obstacles_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        # help prints (Dana)
        count = 0
        for node in nodes:
            if self.occupancy_grid_map[node] == DYN_OBSTACLE:
                count += 1
        return count

    def minimal_distance_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        min_dist = 7.0
        for node in nodes:
            if not self.is_unoccupied(pos=node):
                dist = math.dist(node, global_position)
                if min_dist > dist:
                    min_dist = dist
        if min_dist >= 7.0:
            return 0
        return min_dist

    def average_distance_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        total_distance = 0
        obstacles = []
        for node in nodes:
            if not self.is_unoccupied(pos=node):  # means that it is an obstacle
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

    def largest_angle_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        obstacles = []
        for node in nodes:
            if not self.is_unoccupied(pos=node):  # means that it is an obstacle
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

    def section_of_fail(self): #function to check where the robot fails (Noam)
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

    # We need to see if we should move this function to the second class (Dana)
    def rescan(self, global_position: (int, int)):
        if self.vector==[]:
            s = ('Number of static obstacles', 'Number of dynamic obstacles', 'Minimum distance from obstacle', 'Average distance from obstacles', 'Largest free angle', 'Section of fail')
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
        status = (num_obstacles, num_dynamic_obstacles, min_distance, average_distance, largest_angle, section_of_fail)
        self.vector.append(status)
        #print("vector: ", self.vector)
        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map
    def update_changed_edge_costs(self, local_grid: Dict) -> Vertices:
        vertices = Vertices()
        for node, value in local_grid.items():
            # if obstacle
            if value == OBSTACLE:
                if self.slam_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)
                    self.slam_map.set_obstacle(node)


 #         if value == DYN_OBSTACLE:
 #             if self.slam_map.is_unoccupied(node):
 #                 v = Vertex(pos=node)
 #                 succ = self.slam_map.succ(node)
 #                 for u in succ:
 #                     v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
 #                 vertices.add_vertex(v)
 #                 self.slam_map.set_dynamic_obstacle(node)

 #         elif value == DYN_OBSTACLE_T2:
 #             if self.slam_map.is_unoccupied(node):
 #                 v = Vertex(pos=node)
 #                 succ = self.slam_map.succ(node)
 #                 for u in succ:
 #                     v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
 #                 vertices.add_vertex(v)
 #                 self.slam_map.set_dynamic_obstacle_t2(node)

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
