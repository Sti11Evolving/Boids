from BoidHandler import BoidHandler
from ObstacleHandler import ObstacleHandler
import numpy as np

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.boidHandler = BoidHandler(width, height)
        self.obstacleHandler = ObstacleHandler()

    def add_boid(self, x, y):
        self.boidHandler.add_boid(x, y)

    def add_random_boids(self, num_boids):
        self.boidHandler.add_random_boids(num_boids, self.width, self.height)

    def update(self, dt):
        self.boidHandler.update(self.obstacleHandler, dt)

    def add_walls(self):
        self.obstacleHandler.add_obstacle((0, 0), (self.width, 0))
        self.obstacleHandler.add_obstacle((0, 0), (0, self.height))
        self.obstacleHandler.add_obstacle((0, self.height), (self.width, self.height))
        self.obstacleHandler.add_obstacle((self.width, 0), (self.width, self.height))

