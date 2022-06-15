# Code By Alex Stettner
# Using Pygame Engine

# TODO: Add obstacle avoidance
# TODO: Add menu
# TODO: Add setting sliders
# TODO: Add additional visual effects


import pygame
from Environment import Environment
from Renderer import *
from pygame.locals import *


class Boids_App:
    def __init__(self):
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = 640, 400
        self.renderer = None
        self.environment = Environment(self.width, self.height)
        self.clock = pygame.time.Clock()
        self.debug = True

    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.renderer = Renderer(self, pygame, self._display_surf)
        self._running = True
        self.environment.add_random_boids(10)
        self.environment.add_walls()

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False

    def on_loop(self, dt):
        self.environment.update(dt)

    def on_render(self):
        self.renderer.draw_solid_background((255, 255, 255))
        self.renderer.draw_boids(self.environment.boidHandler)
        self.renderer.draw_obstacles(self.environment.obstacleHandler)
        if self.debug:
            self.renderer.draw_fps(self.clock)
        self.renderer.update()

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        if self.on_init() == False:
            self._running = False

        while (self._running):
            self.clock.tick()
            dt = self.clock.get_time()
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop(dt)
            self.on_render()
        self.on_cleanup()


if __name__ == "__main__":
    boids = Boids_App()
    boids.on_execute()
