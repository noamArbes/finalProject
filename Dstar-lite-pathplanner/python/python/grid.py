import numpy as np
from utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List
from Dynamic import dynamic_obs
import math
import random
import csv
import pygame

OBSTACLE = 255
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
        # we can define here all the static obstacles (Noam)
        self.occupancy_grid_map[50, 50] = OBSTACLE
        self.occupancy_grid_map[7, 8] = OBSTACLE
        #d = self.set_dynamic_obstacle(20,20)
        self.occupancy_grid_map[40, 40] = DYN_OBSTACLE

        self.occupancy_grid_map[20, 20] = DYN_OBSTACLE
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
        print('hi')

    def set_dynamic_obstacle(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """

        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        print('here')
        self.occupancy_grid_map[row, col] = DYN_OBSTACLE

    def remove_obstacle(self, pos: (int, int)):
        """
        :param pos: position of obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = UNOCCUPIED

    # def local_observation(self, global_position: (int, int), view_range: int = 7) -> Dict:
    #    #This is the function we should change, according to the change of the radius around the robot(that is in gui)
    #    """
    #    :param global_position: position of robot in the global map frame
    #    :param view_range: how far ahead we should look
    #    :return: dictionary of new observations
    #    """
    #    (px, py) = global_position
    #    print(global_position)
    #    #nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
    #    #         for y in range(py - view_range, py + view_range + 1)
    #    #         if self.in_bounds((x, y))]
    #    nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
    #             for y in range(py - view_range, py + view_range + 1)
    #             if self.in_bounds((x, y)) and ((x - px) ** 2 + (y - py) ** 2 <= view_range ** 2)]
    #    return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}

    def local_observation(self, global_position: (int, int), view_range: int = 7) -> Dict:
        (px, py) = global_position
        # saves all the coordinates in the circle range
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        # we need to understand how to check if dynamic or static (Dana)
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else DYN_OBSTACLE for node in nodes}

    def count_obstacles_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        print(global_position)
        nodes = [(x, y) for x in range(px - view_range - 1, px + view_range + 1)
                 for y in range(py - view_range - 1, py + view_range + 1)
                 if self.in_bounds((x, y)) and math.dist((x, y), global_position) <= view_range]
        # help prints (Dana)
        count = 0
        for node in nodes:
            if not self.is_unoccupied(pos=node):
                count += 1
        return count

    def minimal_distance_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        print(global_position)
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
        print(global_position)
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
                print("dist", distance)
                total_distance += distance
        num_obstacles = len(obstacles)
        if num_obstacles > 1:
            average_distance = total_distance / (num_obstacles * (num_obstacles - 1) / 2)
            return average_distance
        return 0

    def largest_angle_local_observation(self, global_position: (int, int), view_range: int = 7):
        (px, py) = global_position
        print(global_position)
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


# import math

# def local_observation(self, global_position: (int, int), view_range: int = 70) -> Dict:
# This function cuts out the nodes that are out of the range of 70, but it causes problems in the main
#   """
#  :param global_position: position of robot in the global map frame
# :param view_range: how far ahead we should look
#:return: dictionary of new observations
# """
# (px, py) = global_position
# nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
#        for y in range(py - view_range, py + view_range + 1)
#       if self.in_bounds((x, y))]
# nodes = [node for node in nodes if math.sqrt((node[0] - px) ** 2 + (node[1] - py) ** 2) <= view_range]
# return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}


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

    def rescan(self, global_position: (int, int)):

        # rescan local area
        local_observation = self.ground_truth_map.local_observation(global_position=global_position,
                                                                    view_range=self.view_range)
        num_obstacles = self.ground_truth_map.count_obstacles_local_observation(global_position=global_position,
                                                                                view_range=self.view_range)
        #print(num_obstacles)
        min_distance = self.ground_truth_map.minimal_distance_local_observation(global_position=global_position,
                                                                                view_range=self.view_range)
        #print("minimun distance:", min_distance)
        average_distance = self.ground_truth_map.average_distance_local_observation(global_position=global_position,
                                                                                    view_range=self.view_range)
        #print("average distance:", average_distance, min_distance)
        largest_angle = self.ground_truth_map.largest_angle_local_observation(global_position=global_position,
                                                                              view_range=self.view_range)
        status = (num_obstacles, min_distance, average_distance, largest_angle)
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

            if value == DYN_OBSTACLE:
                if self.slam_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)
                    self.slam_map.set_dynamic_obstacle(node)

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
