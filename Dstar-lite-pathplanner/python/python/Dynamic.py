from priority_queue import PriorityQueue, Priority
import numpy as np
from utils import heuristic, Vertex, Vertices
from typing import Dict, List
import random
import pygame


DYN_OBSTACLE = 100

UNOCCUPIED = 0
OBSTACLE = 255

class dynamic_obs:
    def __init__(self, x, y):
        self.x=x
        self.y=y

    def set_dynamic(self, x, y):

        (row, col) = (x, y)
        #print('here')
#        pygame.draw.circle(self.screen, (0,0,0), (x,y), 70)




