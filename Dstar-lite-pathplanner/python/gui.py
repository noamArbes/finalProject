import pygame
import random
import time
from grid import OccupancyGridMap
from typing import List

# Define some colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)  # BLACK
UNOCCUPIED = (255, 255, 255)  # WHITE
GOAL_A = (0, 0, 255)  # BLUE
GOAL_B = (0, 255, 0)  # GREEN
GOAL_C = (255, 165, 0)  # ORANGE ( Noam)
GOAL_DYN = (0, 255, 0)  # GREEN (Noam)
START = (255, 0, 0)  # RED
GRAY1 = (145, 145, 102)  # GRAY1
OBSTACLE = (77, 77, 51)  # GRAY2
OBSTACLE = (77, 77, 51)  # GRAY2
OBSTACLE_Z1 = (255, 255, 255)  # WHITE
OBSTACLE_Z2 = (255, 255, 255)  # WHITE
DYN_OBSTACLE = (0, 0, 0)  # BLACK
DYN_OBSTACLE_T2 = (0, 0, 0)  # BLACK
LOCAL_GRID = (0, 0, 80)  # BLUE

# Start timer
clock = pygame.time.Clock()
total_time = 0


colors = {
    0: UNOCCUPIED,
    1: GOAL_B,
    2: GOAL_C,
    3: GOAL_A,
    4: GOAL_DYN,
    255: OBSTACLE,
    80: OBSTACLE_Z1,
    20: OBSTACLE_Z2,
    100: DYN_OBSTACLE,
    150: DYN_OBSTACLE_T2
}


class Animation:
    def __init__(self,
                 title="Simulation",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=100,
                 y_dim=50,
                 start=(0, 0),
                 startDyn=(1,1),#Noam
                 goalB=(50, 50),
                 goalC=(51, 51),
                 goalA=(52, 52),
                 goalDyn=(53,53), #Noam
                 viewing_range=3,
                 counter_runs=4
                 ):

        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.startDyn = startDyn #Noam
        self.currentDyn= startDyn #Noam
        self.current = start
        self.observation = {"pos": None, "type": None}
        self.goalB = goalB
        self.goalC = goalC
        self.goalA = goalA
        self.goalDyn = goalDyn
        self.counter_runs = counter_runs
        self.viewing_range = viewing_range
        self.clock = pygame.time.Clock()
        self.total_time = 0
        self.cont = False  # if true - long press on space continue the movement, also backspace set to true(Dana)
        self.counter = 0
        self.counter_t2 = 0
        pygame.init()

        # Set the 'width' and 'height' of the screen
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]

        self.screen = pygame.display.set_mode(window_size)


        # create occupancy grid map
        """
        set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        """
        self.world = OccupancyGridMap(x_dim=x_dim,
                                      y_dim=y_dim,
                                      exploration_setting='8N')

        # Set title of screen
        pygame.display.set_caption(title)

        # set font
        pygame.font.SysFont('Comic Sans MS', 36)

        # Loop until the user clicks the close button
        self.done = False

        # used to manage how fast the screen updates
        self.clock = pygame.time.Clock()

    def get_position(self):
        return self.current

    def set_position(self, pos: (int, int)):
        self.current = pos

    def get_positionDyn(self): #Noam
        return self.currentDyn

    def set_positionDyn(self, pos: (int, int)): #Noam
        self.currentDyn = pos

    def get_goalB(self):
            return self.goalB

    def set_goalB(self, goalB: (int, int)):
        self.goalB = goalB

    def get_goalA(self):
            return self.goalA

    def set_goalA(self, goalA: (int, int)):
        self.goalA = goalA

    def get_goalDyn(self): #Noam
            return self.goalDyn

    def set_goalDyn(self, goalDyn: (int, int)): #Noam
            self.goalDyn = goalDyn


    def get_goalC(self):
        return self.goalC

    def set_goalC(self, goalC: (int, int)):
        self.goalC = goalC

    def set_start(self, start: (int, int)):
        self.start = start

    def display_path(self, path=None):
        if path is not None:
            for step in path:
                # draw a moving robot, based on current coordinates
                step_center = [round(step[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                               round(step[0] * (self.height + self.margin) + self.height / 2) + self.margin]

                # draw robot position as red circle
                pygame.draw.circle(self.screen, START, step_center, round(self.width / 2) - 2)


    def display_obs(self, observations=None): # not sure if in use (Dana)
        if observations is not None:
            for o in observations:
                pygame.draw.rect(self.screen, GRAY1, [(self.margin + self.width) * o[1] + self.margin,
                                                      (self.margin + self.height) * o[0] + self.margin,

                                                       self.width,
                                                      self.height])


    def run_game(self, path_robot=None, path_obstacle=None):
        self.total_time += clock.tick(10) #Speed of the simulation (Noam)
        if path_robot is None:
            path = []

    #automatic (Dana)
        if path_obstacle:
            (x, y) = path_obstacle[1]
            self.set_positionDyn((x, y))

        grid_cell = None
        for event in pygame.event.get():

            if event.type == pygame.QUIT:  # if user clicked close
                print("quit")
                self.done = True  # flag that we are done so we can exit loop
            # manual (Dana)
            elif (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE) or self.cont:
                # space bar pressed. call next action
                if path_robot:
                    (x, y) = path_robot[1]
                    self.set_position((x, y))


            elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
                #print("backspace automates the press space")
                if not self.cont:
                    self.cont = True
                else:
                    self.cont = False


            # set obstacle by holding left-click
            elif pygame.mouse.get_pressed()[0]:
                # User clicks the mouse. Get the position
                (col, row) = pygame.mouse.get_pos()

                # change the x/y screen coordinates to grid coordinates
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)

                # turn pos into cell
                grid_cell = (x, y)

                # set the location in the grid map
                if self.world.is_unoccupied(grid_cell):
                    self.world.set_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": OBSTACLE}


            # remove obstacle by holding right-click
            elif pygame.mouse.get_pressed()[2]:
                # User clicks the mouse. Get the position
                (col, row) = pygame.mouse.get_pos()

                # change the x/y screen coordinates to grid coordinates
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)

                # turn pos into cell
                grid_cell = (x, y)

                # set the location in the grid map
                if not self.world.is_unoccupied(grid_cell):
                    #print("grid cell: ".format(grid_cell))
                    self.world.remove_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": UNOCCUPIED}

        # set the screen background
        self.screen.fill(BLACK)

        # draw the grid
        for row in range(self.x_dim):
            for column in range(self.y_dim):
                # color the cells (background)
                pygame.draw.rect(self.screen, colors[self.world.occupancy_grid_map[row][column]],
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])


  # Old code for dynamic obstacles (Noam)
  #     if self.counter == 2:
  #         for row in range(self.x_dim):
  #             for column in range(self.y_dim):
  #                 # color the cells (background)
  #                 if self.world.occupancy_grid_map[row][column] == 100:
  #                     # set the location in the grid map
  #                     r = row - 1
  #                     c = column
  #                     grid_cell = (r, c)
  #                     remove_cell = (row,column)
  #                     if self.world.is_unoccupied(grid_cell):
  #                         #print(grid_cell)
  #                         self.world.set_dynamic_obstacle(grid_cell)
  #                         self.observation = {"pos": grid_cell, "type": DYN_OBSTACLE}
  #                         self.world.remove_obstacle(remove_cell)
  #                         self.observation = {"pos": remove_cell, "type": UNOCCUPIED}
  #                         self.counter = 0
  #     self.counter += 1

  #     if self.counter_t2 == 2:
  #         for row in range(self.x_dim):
  #             for column in range(self.y_dim):
  #                 # color the cells (background)
  #                 if self.world.occupancy_grid_map[row][column] == 150:
  #                     # set the location in the grid map
  #                     r = row - 1
  #                     c = column - 1
  #                     grid_cell = (r, c)
  #                     remove_cell = (row,column)
  #                     if self.world.is_unoccupied(grid_cell):
  #                         #print(grid_cell)
  #                         self.world.set_dynamic_obstacle_t2(grid_cell)
  #                         self.observation = {"pos": grid_cell, "type": DYN_OBSTACLE_T2}
  #                         self.world.remove_obstacle(remove_cell)
  #                         self.observation = {"pos": remove_cell, "type": UNOCCUPIED}
  #                         self.counter_t2 = 0
  #     self.counter_t2 += 1


        self.display_path(path=path_robot)  # if we disable this the path will disappear (Dana)

        # fill in the goalB cell with green
        pygame.draw.rect(self.screen, GOAL_B, [(self.margin + self.width) * self.goalB[1] + self.margin,
                                               (self.margin + self.height) * self.goalB[0] + self.margin,
                                               self.width,
                                               self.height])
        # fill in the goalC cell with orange (Noam)
        pygame.draw.rect(self.screen, GOAL_C, [(self.margin + self.width) * self.goalC[1] + self.margin,
                                               (self.margin + self.height) * self.goalC[0] + self.margin,
                                               self.width,
                                               self.height])
        # fill in the goalA cell with blue (Noam)
        pygame.draw.rect(self.screen, GOAL_A, [(self.margin + self.width) * self.goalA[1] + self.margin,
                                               (self.margin + self.height) * self.goalA[0] + self.margin,
                                               self.width,
                                               self.height])
        # fill in the goalDyn cell with green, will blank later on (Noam)
        pygame.draw.rect(self.screen, GOAL_DYN, [(self.margin + self.width) * self.goalDyn[1] + self.margin,
                                               (self.margin + self.height) * self.goalDyn[0] + self.margin,
                                               self.width,
                                               self.height])
        # draw a moving robot, based on current coordinates
        robot_center = [round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(
                            self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]
        # draw a dynamic obstacle, based on current coordinates (Noam)
        dyn_center = [round(self.currentDyn[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(
                            self.currentDyn[0] * (self.height + self.margin) + self.height / 2) + self.margin]

        # draw robot position as red circle
        pygame.draw.circle(self.screen, START, robot_center, round(self.width / 2))

        # draw dynamic obstacle position as Black circle (Noam)
        pygame.draw.circle(self.screen, BLACK, dyn_center, round(self.width / 2))


        # draw robot local grid map (viewing range - changed to a circle, need to change the obstacle viewer to the radius we defined)
     #   pygame.draw.rect(self.screen, LOCAL_GRID,
      #                   [robot_center[0] - self.viewing_range * (self.height + self.margin),
       #                   robot_center[1] - self.viewing_range * (self.width + self.margin),
        #                  2 * self.viewing_range * (self.height + self.margin),
         #                 2 * self.viewing_range * (self.width + self.margin)], 2)

        # Draws the radius of the observation area of the robot (Noam)
        pygame.draw.circle(surface=self.screen, color=BLACK, center=robot_center, radius=70.0, width=2)

        # set game tick
        self.clock.tick(60)  #changed to 60 like we told Tal (Dana)

        # go ahead and update screen with that we've drawn
        pygame.display.flip()

    # be 'idle' friendly. If you forget this, the program will hang on exit
    pygame.quit()
