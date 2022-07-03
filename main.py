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
import pygame_menu
import MenuHandler
import os
from SettingsValues import SettingsValues as SV

pygame.display.set_caption('Boids!')


def on_cleanup():
    pygame.quit()


class BoidsApp:
    def __init__(self):
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = 640, 400
        self.fps_limit = 60     # 0 is unlimited
        self.renderer = None
        self.environment = Environment(self.width, self.height)
        self.clock = pygame.time.Clock()
        self.show_fps = True
        self.in_menu = True
        self.menu = None

    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.renderer = Renderer(self, pygame, self._display_surf)
        self._running = True
        self.menu = MenuHandler.main_menu(self.width, self.height)
        self.environment.add_walls()

    def update_settings(self, settings):
        if settings['screen_mode'] == SV.FULL_SCREEN:
            print("fullscreen")
            pygame.display.quit()
            pygame.display.init()
            self._display_surf = pygame.display.set_mode((0, 0), pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.FULLSCREEN)
            self.renderer.update_surface(self._display_surf)
        elif settings['screen_mode'] == SV.BORDERLESS:
            print('borderless')
            os.environ['SDL_VIDEO_WINDOW_POS'] = "0,0"
            pygame.display.quit()
            pygame.display.init()
            self._display_surf = pygame.display.set_mode((0, 0), pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.NOFRAME)
            self.renderer.update_surface(self._display_surf)
        elif settings['screen_mode'] == SV.WINDOWED:
            print('windowed')
            os.environ['SDL_VIDEO_WINDOW_POS'] = ""
            pygame.display.quit()
            pygame.display.init()
            self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.renderer.update_surface(self._display_surf)

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
        elif event.type == MenuHandler.START:
            self.environment.boidHandler.clear_boids()
            self.environment.add_random_boids(25)
            self.in_menu = False
        elif event.type == pygame.KEYDOWN:
            if event.key == 27:  # escape key
                self.in_menu = True
        elif event.type == MenuHandler.UPDATE_SETTINGS:
            self.update_settings(MenuHandler.settings.settings)

    def on_game_loop(self):
        self.clock.tick(self.fps_limit)
        dt = self.clock.get_time()
        self.environment.update(dt)

    def on_game_render(self):
        self.renderer.draw_solid_background((255, 255, 255))
        self.renderer.draw_boids(self.environment.boidHandler)
        self.renderer.draw_obstacles(self.environment.obstacleHandler)
        if self.show_fps:
            self.renderer.draw_fps(self.clock)
        self.renderer.update()

    def on_execute(self):
        if self.on_init() is False:
            self._running = False

        while self._running:
            events = pygame.event.get()
            for event in events:
                self.on_event(event)

            if self.in_menu:
                self.menu.update(events)
                self.renderer.draw_menu(self.menu)
            else:
                self.on_game_loop()
                self.on_game_render()

        on_cleanup()


if __name__ == "__main__":
    boids = BoidsApp()
    boids.on_execute()
