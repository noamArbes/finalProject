from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
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
    start = (15,15)
    goalB = (32, 19)
    goalC = (17, 45)
    goalA = (15, 15)
    goalDyn = []
    startDyn = []
    new_pos_dyn = []
    last_pos_dyn = []
    path_list = []
    unoccupied_array = []
    counter = []


    view_range = 3
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
    unoccupied_hall_left = new_map.find_zeros_in_hall_left()
    unoccupied_hall_right = new_map.find_zeros_in_hall_right()
    for i in range(1, global_var.num_of_dyn_obs + 1):
        start_side = random.random()
        if start_side <= 0.5:
            s = random.choice(unoccupied_hall_right)
            startDyn.append(s)
            will_enter_room = random.random()
            if will_enter_room <= 0.8:
                g = random.choice(unoccupied_hall_left)
            else:
                g = random.choice(unoccupied_array)
            goalDyn.append(g)
        else:
            s = random.choice(unoccupied_hall_left)
            startDyn.append(s)
            will_enter_room = random.random()
            if will_enter_room <= 0.8:
                g = random.choice(unoccupied_hall_right)
            else:
                g = random.choice(unoccupied_array)
            goalDyn.append(g)

    '''
    for i in range(1, global_var.num_of_dyn_obs + 1):
        will_enter_room = random.random()
        if will_enter_room <= 0.8:
            g = random.choice(unoccupied_hall_left)
            goalDyn.append(g)
        else:
            g = random.choice(unoccupied_array)
            goalDyn.append(g)
    '''
    new_position = start
    last_position = start

    # Setting the dynamic obstacles start position array (Dana)
    for i in range(1, global_var.num_of_dyn_obs + 1):
        new_pos_dyn.append(startDyn[i-1])
        last_pos_dyn.append(startDyn[i-1])
        counter.append(0)

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

    def Robot_movement(last_position, dstar):

        new_position = gui.current
        new_observation = gui.observation
        new_map = gui.world
        # d star
        if new_observation is not None:
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)
        if new_position != last_position:
            last_position = new_position
            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

        return old_map, last_position, new_position

    def wait(counter):
        if counter < 10:
            counter += 1
        else:
            counter = 0
        return counter
    def Dynamic_obs_movement():
        for i in range(1, global_var.num_of_dyn_obs + 1):
            gui.world.remove_obstacle(new_pos_dyn[i - 1])  # Remove the value from the gui map (Dana)
            dstar_list[i - 1].sensed_map.remove_obstacle(new_pos_dyn[i - 1])  # Remove the value from the grid map (g) (Dana)
            new_pos_dyn[i - 1] = gui.currentDyn[i - 1]
            gui.world.set_dynamic_obstacle(new_pos_dyn[i - 1])  # Setting the value in the gui map (Dana)
            dstar_list[i - 1].sensed_map.set_dynamic_obstacle(new_pos_dyn[i - 1]) # Setting the value in the grid map (g) (Dana)
        for i in range(1, global_var.num_of_dyn_obs + 1):
            if new_pos_dyn[i - 1] != goalDyn[i - 1]:
                path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])
            else:
                counter[i - 1] = wait(counter[i-1])
                if counter[i - 1] == 10:
                    path_list[i - 1] = []
                    will_enter_room = random.random()
                    if will_enter_room <= 0.8:
                        if goalDyn[i - 1] in unoccupied_hall_right:
                            goal = random.choice(unoccupied_hall_left)
                        else:
                            goal = random.choice(unoccupied_hall_right)
                    else:
                        goal = random.choice(unoccupied_array)
                    goalDyn[i - 1] = goal

                    dstar_obj = DStarLite(map=new_map,
                                          s_start=last_pos_dyn[i - 1],
                                          s_goal=goalDyn[i - 1],
                                          value=DYN_OBSTACLE)
                    dstar_list[i - 1] = dstar_obj
                    path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])

    '''


    def Dynamic_obs_movement():
        for i in range(1, global_var.num_of_dyn_obs + 1):
            neighbors = slam.dyn_obs_neighbors(new_pos_dyn[i - 1])
            for n in neighbors:
               gui.world.remove_obstacle(n)  # Remove the value from the gui map (Dana)
               dstar_list[i - 1].sensed_map.remove_obstacle(n)  # Remove the value from the grid map (g) (Dana)
            #  שלב ראשון פה יהיה למצוא מי השכנים דרך קריאה לפונקציה ששולחת את הערך של new_pos_dyn[i - 1] ומחזירה את כל השכנים מסביב - זהה לפונקציה local_observation בגריד
            # אחכ את יכולה להוסיף את הלולאה של ההסרה שתרוץ על כל השכנים (הסינטקס יהיה ממש זהה לשתי השורות למטה, רק להכניס ללולאה והערך שיהיה בסוגריים במקום new_pos_dyn יהיה הערך של אותו שכן)
            gui.world.remove_obstacle(new_pos_dyn[i - 1])  # Remove the value from the gui map (Dana)
            dstar_list[i - 1].sensed_map.remove_obstacle(new_pos_dyn[i - 1])  # Remove the value from the grid map (g) (Dana)
            new_pos_dyn[i - 1] = gui.currentDyn[i - 1]
            neighbors = slam.dyn_obs_neighbors(new_pos_dyn[i - 1])
            for n in neighbors:
               gui.world.set_dynamic_obstacle_neighbors(n)  # Setting the value in the gui map (Dana)
               dstar_list[i - 1].sensed_map.set_dynamic_obstacle_neighbors(n)  # Setting the value in the grid map (g) (Dana)
            # פה תצטרכי לקרוא שוב לפונקציה שמוצאת את השכנים ולעדכן את השכנים
            # כאן את יכולה להוסיף את הלולאה של הצבת הערך שתרוץ על כל השכנים (הסינטקס יהיה ממש זהה לשתי השורות למטה, רק להכניס ללולאה והערך שיהיה בסוגריים במקום new_pos_dyn יהיה הערך של אותו שכן)
            gui.world.set_dynamic_obstacle(new_pos_dyn[i - 1])  # Setting the value in the gui map (Dana)
            dstar_list[i - 1].sensed_map.set_dynamic_obstacle(
                new_pos_dyn[i - 1])  # Setting the value in the grid map (g) (Dana)

        for i in range(1, global_var.num_of_dyn_obs + 1):
            if new_pos_dyn[i - 1] != goalDyn[i - 1]:
                path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])
            else:
                path_list[i - 1] = []
                will_enter_room = random.random()
                if will_enter_room <= 0.8:
                    if goalDyn[i - 1] in unoccupied_hall_right:
                        g = random.choice(unoccupied_hall_left)
                    else:
                        g = random.choice(unoccupied_hall_right)
                else:
                    g = random.choice(unoccupied_array)
                goalDyn[i - 1] = g

                dstar_obj = DStarLite(map=new_map,
                                      s_start=last_pos_dyn[i - 1],
                                      s_goal=goalDyn[i - 1],
                                      value=DYN_OBSTACLE)
                dstar_list[i - 1] = dstar_obj
                path_list[i - 1], g, rhs = dstar_list[i - 1].move_and_replan_dyn(obstacle_position=new_pos_dyn[i - 1])
    '''
    # Run simulation
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

            Dynamic_obs_movement()
            old_map, last_position, new_position = Robot_movement(last_position, dstar1)

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]

            if new_position == goalB:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = False
                global_var.arrivedA = False
                global_var.arrivedC = False

        # navigation to goalC (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == False and gui.done == False:

            path, g, rhs = dstar2.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            Dynamic_obs_movement()
            old_map, last_position, new_position = Robot_movement(last_position, dstar2)

            if new_position == goalC:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = False
                global_var.arrivedA = False
                global_var.arrivedC = True

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]

        # navigate to goalB the second time (Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == False \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar1.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            Dynamic_obs_movement()
            old_map, last_position, new_position = Robot_movement(last_position, dstar1)

            if new_position == goalB:
                global_var.arrivedB1 = True
                global_var.arrivedB2 = True
                global_var.arrivedA = False
                global_var.arrivedC = True

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]

        # navigate back to goalA(Noam)
        while global_var.arrivedA == False and global_var.arrivedB1 == True and global_var.arrivedB2 == True \
                and global_var.arrivedC == True and gui.done == False:

            path, g, rhs = dstar3.move_and_replan(robot_position=new_position)
            gui.run_game(path_robot=path, path_obstacle=path_list)

            Dynamic_obs_movement()
            old_map, last_position, new_position = Robot_movement(last_position, dstar3)

            if new_position == goalA:
                global_var.arrivedA = True
                global_var.arrivedB1 = False
                global_var.arrivedB2 = False
                global_var.arrivedC = False
                global_var.counter_runs = global_var.counter_runs + 1

            for i in range(1, global_var.num_of_dyn_obs + 1):
                if new_pos_dyn[i-1] != last_pos_dyn[i-1]:
                    last_pos_dyn[i-1] = new_pos_dyn[i-1]

    # export csv file (Dana)
    with open('slam.vector.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write the vector to the CSV file
        writer.writerows(slam.vector)
    os.startfile("slam.vector.csv")
