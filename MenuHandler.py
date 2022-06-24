import pygame
import pygame_menu


class MenuHandler:
    def __init__(self):
        self.START = pygame.event.custom_type()
        self.RETURN_TO_MENU = pygame.event.custom_type()

    def main_menu(self, width, height):
        main_menu = pygame_menu.menu.Menu('Boids!', width, height, mouse_motion_selection=True)
        main_menu.add.button('Start', lambda: pygame.event.post(pygame.event.Event(self.START)))
        main_menu.add.button('Settings', pygame_menu.events.NONE)
        main_menu.add.button('Quit', pygame_menu.events.PYGAME_QUIT)
        return main_menu
