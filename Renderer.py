class Renderer:
    def __init__(self, app, pygame, surface):
        self.app = app
        self.pygame = pygame
        self.screen = surface

    def update_surface(self, surface):
        self.screen = surface

    def draw_menu(self, menu):
        menu.draw(self.screen)
        self.update()

    def draw_solid_background(self, color):
        self.pygame.draw.rect(self.screen, color, self.pygame.Rect((0, 0), self.app.size))

    def draw_boids(self, boid_handler):
        boids = boid_handler.get_boids()
        for boid in boids:
            self.pygame.draw.circle(self.screen, boid['color'], (int(boid['x']), int(boid['y'])), 10)

    def draw_obstacles(self, obstacles_handler):
        for obstacle in range(obstacles_handler.num_obstacles):
            self.pygame.draw.line(self.screen, (0, 0, 0), obstacles_handler.obstacles[obstacle][0],
                                  obstacles_handler.obstacles[obstacle][1], width=10)

    def draw_fps(self, clock):
        font = self.pygame.font.SysFont(None, 24)
        img = font.render('FPS: ' + str(int(clock.get_fps())), True, (0, 0, 0))
        self.screen.blit(img, (20, 20))

    def update(self):
        self.pygame.display.flip()
