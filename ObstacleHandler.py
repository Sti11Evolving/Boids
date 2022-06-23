import numpy as np


class ObstacleHandler:
    def __init__(self):
        self.obstacles = np.array([[]])
        self.num_obstacles = 0

    def clear_obstacles(self):
        self.obstacles = np.array([[]])
        self.num_obstacles = 0

    def add_obstacle(self, point1, point2):
        if self.num_obstacles > 0:
            self.obstacles = np.concatenate((self.obstacles, [[point1, point2]]), axis=0)
        else:
            self.obstacles = np.array([[point1, point2]])
            self.num_obstacles += 1
