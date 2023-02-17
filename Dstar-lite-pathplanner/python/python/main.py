from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
from Dynamic import dynamic_obs
import pandas as pd

import csv
import os
import pygame
import random


OBSTACLE = 255
DYN_OBSTACLE = 100
UNOCCUPIED = 0

if __name__ == '__main__':

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 80
    y_dim = 100
    start = (10, 10)
    goal = (40, 70)
    view_range = 7


    gui = Animation(title="Simulation",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map

    new_position = start
    last_position = start

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar = DStarLite(map=new_map,
                      s_start=start,
                      s_goal=goal)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)

    # move and compute path
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)
    
    dyn = dynamic_obs(x=30,y=30)

#    Dana = moving_obs(map = new_map)

    while not gui.done:
        # update the map
        # print(path)
        # drive gui
        gui.run_game(path=path)
        dyn.set_dynamic(30,30)

        new_position = gui.current
        new_observation = gui.observation
        new_map = gui.world



        """
        if new_observation is not None:
            if new_observation["type"] == OBSTACLE:
                dstar.global_map.set_obstacle(pos=new_observation["pos"])
            if new_observation["pos"] == UNOCCUPIED:
                dstar.global_map.remove_obstacle(pos=new_observation["pos"])
        """
        if new_position == goal:  # added to prevent error at the end (Dana)
            gui.done = True

        if new_observation is not None:
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)

        if new_position != last_position:
            last_position = new_position

            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
    print("Session time: " + str(gui.total_time / 1000) + " seconds")  # converts time to seconds
    print(slam.vector)

# export csv file (Dana)
    with open('slam.vector.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write the vector to the CSV file
        writer.writerows(slam.vector)
    os.startfile("slam.vector.csv")

