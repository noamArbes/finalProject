from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
from Dynamic import dynamic_obs
import pandas as pd
import global_var
import csv
import os
import pygame
import random

#Check if to delete- don't think it's in use (Noam)
OBSTACLE = 255
DYN_OBSTACLE = 100
DYN_OBSTACLE_T2 = 150
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
    start = (16, 16) # changed (Noam)
    goalB = (60, 45)  # green goal (Noam)
    goalC = (25, 70)  # orange goal (Noam)
    goalA = (16, 16)  # blue goal (Noam)
    goalDyn = (60, 85) # Noam
    startDyn = (15, 17) #Noam
    view_range = 7
    global_var.arrivedA = True
    global_var.arrivedB1 = False
    global_var.arrivedB2 = False
    global_var.arrivedC = False
    counter_runs = 0
    gui = Animation(title="Simulation",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    startDyn=startDyn,
                    goalA=goalA,
                    goalB=goalB,
                    goalC=goalC,
                    goalDyn=goalDyn,
                    counter_runs=counter_runs,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map

    new_position = start
    last_position = start
    new_position_dyn = startDyn
    last_position_dyn = startDyn

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar1 = DStarLite(map=new_map,
                       s_start=start,
                       s_goal=goalB)
    # D* Lite goal C(Noam)
    dstar2 = DStarLite(map=new_map,
                     s_start=start,
                     s_goal=goalC)
    # D* Lite goal A(Noam)
    dstar3 = DStarLite(map=new_map,
                     s_start=start,
                     s_goal=goalA)

    dstar4 = DStarLite(map=new_map,
                     s_start=startDyn,
                     s_goal=goalDyn)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)



    while counter_runs < 3 and gui.done == False:
        if gui.done == True:
            pygame.quit()
        # move and compute path to the first path (Noam)
        path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
        path_obstacle, g, rhs = dstar4.move_and_replan_dyn(obstacle_position=new_position_dyn)

        # first navigation to goalB (Noam)
        while  global_var.arrivedA == True and global_var.arrivedB1 == False and global_var.arrivedB2 == False and \
                global_var.arrivedC == False and gui.done == False:

            path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_obstacle)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            #Dynamic obs movement
            new_position_dyn= gui.currentDyn
            path_obstacle, g, rhs = dstar4.move_and_replan_dyn(obstacle_position=new_position_dyn)
            if new_position_dyn == gui.goalDyn:
                path_obstacle =[]

            if new_position == goalB:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = False
                global_var.arrivedA = False
                global_var.arrivedC = False
                # d star
            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)
            if new_position != last_position:
                last_position = new_position
                # slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar1.new_edges_and_old_costs = new_edges_and_old_costs
                dstar1.sensed_map = slam_map

        #navigation to goalC (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False\
                and global_var.arrivedC == False and gui.done == False:

            path, g, rhs = dstar2.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_obstacle)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            #Dynamic obs movement
            new_position_dyn= gui.currentDyn
            path_obstacle, g, rhs = dstar4.move_and_replan_dyn(obstacle_position=new_position_dyn)
            if new_position_dyn == gui.goalDyn:
                path_obstacle =[]

            if new_position == goalC:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = False
                global_var.arrivedA = False
                global_var.arrivedC = True
            # d star
            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar2.new_edges_and_old_costs = new_edges_and_old_costs
                dstar2.sensed_map = slam_map

        #navigate to goalB the second time (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_obstacle)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            #Dynamic obs movement
            new_position_dyn = gui.currentDyn
            path_obstacle, g, rhs = dstar4.move_and_replan_dyn(obstacle_position=new_position_dyn)
            if new_position_dyn == gui.goalDyn:
                path_obstacle = []

            if new_position == goalB:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = True
                global_var.arrivedA = False
                global_var.arrivedC = True

            # d star
            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar1.new_edges_and_old_costs = new_edges_and_old_costs
                dstar1.sensed_map = slam_map

        #navigate back to goalA(Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == True \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar3.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_obstacle)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            #Dynamic obs movement
            new_position_dyn = gui.currentDyn
            path_obstacle, g, rhs = dstar4.move_and_replan_dyn(obstacle_position=new_position_dyn)
            if new_position_dyn == gui.goalDyn:
                path_obstacle = []

            if new_position == goalA:
                global_var.arrivedA = True
                global_var.arrivedB1 = False
                global_var.arrivedB2 = False
                global_var.arrivedC = False
                counter_runs = counter_runs + 1

            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar3.new_edges_and_old_costs = new_edges_and_old_costs
                dstar3.sensed_map = slam_map
    
# export csv file (Dana)
    with open('slam.vector.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write the vector to the CSV file
        writer.writerows(slam.vector)
    os.startfile("slam.vector.csv")