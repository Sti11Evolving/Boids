import numpy as np
from numpy import linalg as la
import random
from numpy.linalg import norm

# Returns a numpy array of boid nums in a particular radius
# Needs to be given the distances array
# Result will include the boid that distances are calculated from
def get_nearby(distances, radius):
    nearby = np.array([], dtype=int)
    for distance in distances:
        if distance[0] < radius:
            nearby = np.append(nearby, distance[1])
    return nearby

# Probably could be more readable if boids were individual objects, but using numpy arrays for performance's sake
# Holds positional, velocity, and color for all boids in 2D numpy array and updating functions
class BoidHandler:
    def __init__(self, environment_width, environment_height, max_speed=.1):
        self.environment_width = environment_width
        self.environment_height = environment_height

        self._boids_pos = np.empty((0, 2), dtype=int)  # 2D array holding boid positional data
        # First dimension is for each boid
        # Second dimension is x or y
        # ie. boids_pos[0][1] would get the y position for the first boid

        self._boids_vel = np.empty((0, 2), dtype=float)  # 2D array holding boid velocity data
        # First dimension is for each boid
        # Second dimension is x_vel or y_vel
        # ie. boids_pos[1][0] would get the x_vel position for the second boid

        self.boids_color = np.empty((0, 3), dtype=int)  # 2D array holding boid color data
        self.num_boids = 0

        # Constants that determine the boids flocking behavior
        self.max_speed = max_speed
        self.centering_factor = 0.001
        self.min_distance = 50
        self.avoid_factor = 0.1
        self.group_range = 100
        self.matching_force = .5

    # Reset all boid data
    def clear_boids(self):
        self._boids_pos = np.empty((0, 2), dtype=int)
        self._boids_vel = np.empty((0, 2), dtype=float)
        self.boids_color = np.empty((0, 3), dtype=int)
        self.num_boids = 0

    # Add boid data for a new boid to the data list
    def add_boid(self, pos=[[0, 0]], vel=[[0, 0]], color=[[255, 0, 0]]):
        self._boids_pos = np.concatenate((self._boids_pos, pos), dtype=int, axis=0)
        self._boids_vel = np.concatenate((self._boids_vel, vel), dtype=float, axis=0)
        self.boids_color = np.concatenate((self.boids_color, color), axis=0)
        self.num_boids += 1

    # returns a dictionary containing information for a particular boid for debug purposes
    def get_boid(self, boid_num):
        return {
            "x": self._boids_pos[boid_num][0],
            "y": self._boids_pos[boid_num][1],
            "x_vel": self._boids_vel[boid_num][0],
            "y_vel": self._boids_vel[boid_num][1],
            "color": self.boids_color[boid_num],
        }

    def add_random_boids(self, num_boids, env_width, env_height):
        random.seed()
        for boid in range(num_boids):
            self.add_boid([[random.randrange(0, env_width), random.randrange(0, env_height)]],
                          [[random.uniform(-100, 100), random.uniform(-100, 100)]])

    # Returns an array of distances in relation to a particular boid
    def get_distances(self, boid_num):
        distances = np.empty(self.num_boids, dtype=[('distance', int), ('index', int)])
        pos = self._boids_pos[boid_num]
        idx = 0
        for other_pos in self._boids_pos[::, ]:
            distances[idx] = (((pos[0] - other_pos[0]) ** 2 + (pos[1] - other_pos[1]) ** 2) ** .5, idx)
            idx += 1
        return distances

    # Calculate force of boids to turn towards center of group
    def centering_force(self, boid_num, distances):
        group = get_nearby(distances, self.group_range)
        num_neighbors = np.size(group)
        center = np.zeros(2)
        for neighbor in group:
            center += self._boids_pos[neighbor]
        center = center / num_neighbors
        self._boids_vel[boid_num] += (center - self._boids_pos[boid_num]) * self.centering_factor

    # Calculate object avoiding force
    def obstacle_avoid_force(self, boid_num):
        boid_x = self._boids_pos[boid_num][0]
        if boid_x < self.group_range:
            self._boids_vel[boid_num][0] += (self.group_range - boid_x) * self.avoid_factor
        elif boid_x > self.environment_width - self.group_range:
            self._boids_vel[boid_num][0] += ((self.environment_width - self.group_range) - boid_x) * self.avoid_factor

        boid_y = self._boids_pos[boid_num][1]
        if boid_y < self.group_range:
            self._boids_vel[boid_num][1] += (self.group_range - boid_y) * self.avoid_factor
        elif boid_y > self.environment_height - self.group_range:
            self._boids_vel[boid_num][1] += ((self.environment_height - self.group_range) - boid_y) * self.avoid_factor

    # Calculate force avoiding collisions with nearby boids
    def boid_avoid_force(self, boid_num, distances):
        boids_to_avoid = get_nearby(distances, self.min_distance)
        print(f"~~~~ {boid_num} ~~~~")
        if boids_to_avoid.size != 0:
            print(f"{boids_to_avoid}")
            avoid_vel = np.array([0, 0], dtype=float)
            for other_boid in boids_to_avoid:
                if other_boid != boid_num:
                    diff = self._boids_pos[boid_num] - self._boids_pos[other_boid]
                    print(diff / norm(diff, keepdims=True))
                    avoid_vel += (diff / norm(diff, keepdims=True) ** 2)
            self._boids_vel[boid_num] += avoid_vel * self.avoid_factor

    # Calculate force matching velocity with neighboring boids
    def velocity_match_force(self, boid_num, distances):
        group = get_nearby(distances, self.group_range)
        avg_group_vel = np.array([0, 0], dtype=float)
        for other_boid in group:
            avg_group_vel += self._boids_vel[other_boid]
        avg_group_vel /= np.size(group) / 2
        self._boids_vel[boid_num] += avg_group_vel * self.matching_force

    def speed_limiter(self, boid_num):
        if la.norm(self._boids_vel[boid_num]) > self.max_speed:
            print(f"Boid {boid_num} moved too fast")
            self._boids_vel[boid_num] = self.max_speed * (
                        self._boids_vel[boid_num] / la.norm(self._boids_vel[boid_num]))

    def move_boids(self, dt):
        self._boids_pos = np.add(self._boids_pos, np.multiply(self._boids_vel, dt))

    def update(self, obstacles, dt):
        for boid in range(self.num_boids):
            distances = self.get_distances(boid)
            self.centering_force(boid, distances)
            self.boid_avoid_force(boid, distances)
            self.obstacle_avoid_force(boid)
            self.velocity_match_force(boid, distances)
            self.speed_limiter(boid)
        print(self._boids_vel)
        self.move_boids(dt)
