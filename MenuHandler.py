import pygame
import pygame_menu
from SettingsValues import SettingsValues as SV
from Settings import Settings

START = pygame.event.custom_type()
RETURN_TO_MENU = pygame.event.custom_type()
UPDATE_SETTINGS = pygame.event.custom_type()
settings = Settings()


def apply_settings():
    settings.update_settings()
    pygame.event.post(pygame.event.Event(UPDATE_SETTINGS))


def settings_menu(width, height):
    menu = pygame_menu.menu.Menu('Settings', width, height, mouse_motion_selection=True)
    menu.add.selector(
        style=pygame_menu.widgets.SELECTOR_STYLE_FANCY,
        title='screen mode',
        items=[('Full screen', SV.FULL_SCREEN),
               ('Windowed', SV.WINDOWED),
               ('Borderless Window', SV.BORDERLESS)],
        onreturn=settings.set_window_mode,  # User press "Return" button
        onchange=settings.set_window_mode  # User changes value with left/right keys
    )
    menu.add.button('Apply', apply_settings)
    menu.add.button('Back', pygame_menu.events.BACK)
    return menu


def main_menu(width, height):
    menu = pygame_menu.menu.Menu('Boids!', width, height, mouse_motion_selection=True)
    menu.add.button('Start', lambda: pygame.event.post(pygame.event.Event(START)))
    menu.add.button('Settings', settings_menu(width, height))
    menu.add.button('Quit', pygame_menu.events.PYGAME_QUIT)
    return menu
