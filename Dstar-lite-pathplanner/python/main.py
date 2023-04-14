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

# Check if to delete- don't think it's in use (Noam)
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

    x_dim = 41
    y_dim = 61
    start = (9,9)  # changed (Noam)
    goalB = (29, 19)  # green goal (Noam)
    goalC = (17, 45)  # orange goal (Noam)
    goalA = (9,9)  # blue goal (Noam)
    goalDyn = []
    startDyn = []
    new_pos_dyn = []
    last_pos_dyn = []
    path_list = []
    unoccupied_array = []

    view_range = 5
    global_var.arrivedA = True
    global_var.arrivedB1 = False
    global_var.arrivedB2 = False
    global_var.arrivedC = False
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
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map
    # Setting the start anf goal for every dynamic obstacle (Dana)
    unoccupied_array = new_map.find_zeros()
    for i in range(1, global_var.num_of_dyn_obs + 1):
        s = random.choice(unoccupied_array)
        startDyn.append(s)
    for i in range(1, global_var.num_of_dyn_obs + 1):
        g = random.choice(unoccupied_array)
        goalDyn.append(g)
        unoccupied_array.remove(g)

    new_position = start
    last_position = start

    # Setting the dynamic obstacles start position array (Dana)
    for i in range(1, global_var.num_of_dyn_obs + 1):
        new_pos_dyn.append(startDyn[i-1])
        last_pos_dyn.append(startDyn[i-1])

    # D* Lite (optimized)
    dstar1 = DStarLite(map=new_map,
                       s_start=start,
                       s_goal=goalB,
                       value=0)
    # D* Lite goal C(Noam)
    dstar2 = DStarLite(map=new_map,
                       s_start=start,
                       s_goal=goalC,
                       value=0)
    # D* Lite goal A(Noam)
    dstar3 = DStarLite(map=new_map,
                       s_start=start,
                       s_goal=goalA,
                       value=0)
    # Setting all the dynamic obstacles Dstar path (Dana)
    dstar_list = []
    for i in range(1, global_var.num_of_dyn_obs + 1):
        dstar_obj = DStarLite(map=new_map,
                              s_start=startDyn[i-1],
                              s_goal=goalDyn[i-1],
                              value=DYN_OBSTACLE)
        dstar_list.append(dstar_obj)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)

    def Dynamic_obs_movement():
        for i in range(1, global_var.num_of_dyn_obs + 1):
            gui.world.remove_obstacle(new_pos_dyn[i - 1])  # Remove the value from the gui map (Dana)
            dstar_list[i - 1].sensed_map.remove_obstacle(
                new_pos_dyn[i - 1])  # Remove the value from the grid map (g) (Dana)
            new_pos_dyn[i - 1] = gui.currentDyn[i - 1]
            gui.world.set_dynamic_obstacle(new_pos_dyn[i - 1])  # Setting the value in the gui map (Dana)
        for i in range(1, global_var.num_of_dyn_obs + 1):
            if new_pos_dyn[i - 1] != goalDyn[i - 1]:
                path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])
            else:
                path_list[i - 1] = []
                g = random.choice(unoccupied_array)
                goalDyn[i - 1] = g
                unoccupied_array.remove(g)
                dstar_obj = DStarLite(map=new_map,
                                      s_start=last_pos_dyn[i - 1],
                                      s_goal=goalDyn[i - 1],
                                      value=DYN_OBSTACLE)
                dstar_list[i - 1] = dstar_obj
                path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])

    while global_var.counter_runs < 3 and gui.done == False:
        if gui.done == True:
            pygame.quit()
        # move and compute path to the first path (Noam)
        path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
        # Setting all the paths to the dynamic obstacles (Dana)
        if global_var.counter_runs == 0:
            for i in range(1, global_var.num_of_dyn_obs + 1):
                path_obj, g, rhs = dstar_list[i-1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i-1])
                path_list.append(path_obj)

        # first navigation to goalB (Noam)
        while global_var.arrivedA == True and global_var.arrivedB1 == False and global_var.arrivedB2 == False and \
                global_var.arrivedC == False and gui.done == False:

            path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            Dynamic_obs_movement()

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
                # we need to add the dynamic obs slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar1.new_edges_and_old_costs = new_edges_and_old_costs
                dstar1.sensed_map = slam_map

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]


        # navigation to goalC (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == False and gui.done == False:

            path, g, rhs = dstar2.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            # Dynamic obs movement
            Dynamic_obs_movement()

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

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]


        # navigate to goalB the second time (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            Dynamic_obs_movement()

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

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]

        # navigate back to goalA(Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == True \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar3.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            new_position = gui.current
            new_observation = gui.observation
            new_map = gui.world

            Dynamic_obs_movement()

            if new_position == goalA:
                global_var.arrivedA = True
                global_var.arrivedB1 = False
                global_var.arrivedB2 = False
                global_var.arrivedC = False
                global_var.counter_runs = global_var.counter_runs + 1

            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)

            if new_position != last_position:
                last_position = new_position
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                dstar3.new_edges_and_old_costs = new_edges_and_old_costs
                dstar3.sensed_map = slam_map

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]


    # export csv file (Dana)
    with open('slam.vector.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write the vector to the CSV file
        writer.writerows(slam.vector)
    os.startfile("slam.vector.csv")
