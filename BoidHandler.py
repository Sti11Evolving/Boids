import numpy as np
from numpy import linalg as la
import random
from numpy.linalg import norm


# Returns a numpy array of boid nums in a particular radius
# Needs to be given the distances array
# Result will include the boid that distances are calculated from
def _get_nearby(distances, radius):
    nearby = np.array([], dtype=int)
    distances_iter = np.nditer(distances, flags=["c_index"])
    for distance in distances_iter:
        if distance < radius:
            nearby = np.append(nearby, distances_iter.index)
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
        self.centering_factor = 0.00001
        self.min_distance = 50
        self.avoid_factor = 0.1
        self.group_range = 100
        self.obstacle_range = 20
        self.matching_force = .005
        self.speed_mod = 5

    # Reset all boid data
    def clear_boids(self):
        self._boids_pos = np.empty((0, 2), dtype=int)
        self._boids_vel = np.empty((0, 2), dtype=float)
        self.boids_color = np.empty((0, 3), dtype=int)
        self.num_boids = 0

    # Add boid data for a new boid to the data list
    def add_boid(self, pos=[[0, 0]], vel=[[0, 0]], color=[[255, 0, 0]]):
        self._boids_pos = np.concatenate((self._boids_pos, pos), dtype=float, axis=0)
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
    def get_distances(self, boid_num, other_boids):
        distances = np.empty_like(other_boids, dtype=int)
        pos = self._boids_pos[boid_num]
        for idx, other_boid in enumerate(other_boids):
            other_pos = self._boids_pos[other_boid]
            distance = ((pos[0] - other_pos[0]) ** 2 + (pos[1] - other_pos[1]) ** 2) ** .5
            distances[idx] = distance
        return distances

    def distances_matrix(self):
        dist_mat = np.zeros(shape=[self.num_boids, self.num_boids], dtype=int)
        for boid in range(self.num_boids - 1):
            dist = self.get_distances(boid, range(boid, self.num_boids))
            dist_mat[boid::, boid] = dist
            dist_mat[boid][boid::] = dist
        return dist_mat


    # Calculate force of boids to turn towards center of group
    def centering_force(self, boid_num, distances):
        group = _get_nearby(distances, self.group_range)
        num_neighbors = np.size(group)
        center = np.zeros(2)
        for neighbor in group:
            center += self._boids_pos[neighbor]
        center = center / num_neighbors
        self._boids_vel[boid_num] += (center - self._boids_pos[boid_num]) * self.centering_factor

    # Calculate object avoiding force
    def obstacle_avoid_force(self, boid_num):
        boid_x = self._boids_pos[boid_num][0]
        if boid_x < self.obstacle_range:
            self._boids_vel[boid_num][0] += (self.obstacle_range - boid_x) * self.avoid_factor
        elif boid_x > self.environment_width - self.obstacle_range:
            self._boids_vel[boid_num][0] += ((self.environment_width - self.obstacle_range) - boid_x) * self.avoid_factor

        boid_y = self._boids_pos[boid_num][1]
        if boid_y < self.obstacle_range:
            self._boids_vel[boid_num][1] += (self.obstacle_range - boid_y) * self.avoid_factor
        elif boid_y > self.environment_height - self.obstacle_range:
            self._boids_vel[boid_num][1] += ((self.environment_height - self.obstacle_range) - boid_y) * self.avoid_factor

    # Calculate force avoiding collisions with nearby boids
    def boid_avoid_force(self, boid_num, distances):
        boids_to_avoid = _get_nearby(distances, self.min_distance)
        if boids_to_avoid.size != 0:
            avoid_vel = np.array([0, 0], dtype=float)
            for other_boid in boids_to_avoid:
                if other_boid != boid_num:
                    diff = self._boids_pos[boid_num] - self._boids_pos[other_boid]
                    avoid_vel += (diff / norm(diff, keepdims=True) ** 2)
            self._boids_vel[boid_num] += avoid_vel * self.avoid_factor

    # Calculate force matching velocity with neighboring boids
    def velocity_match_force(self, boid_num, distances):
        group = _get_nearby(distances, self.group_range)
        avg_group_vel = np.array([0, 0], dtype=float)
        for other_boid in group:
            avg_group_vel += self._boids_vel[other_boid]
        avg_group_vel /= np.size(group) / 2
        self._boids_vel[boid_num] += avg_group_vel * self.matching_force

    def speed_limiter(self, boid_num):
        if la.norm(self._boids_vel[boid_num]) > self.max_speed:
            self._boids_vel[boid_num] = self.max_speed * (
                        self._boids_vel[boid_num] / la.norm(self._boids_vel[boid_num]))

    def move_boids(self, dt):
        move_amount = np.multiply(self._boids_vel, dt * self.speed_mod)
        self._boids_pos = np.add(self._boids_pos, move_amount)

    def update(self, obstacles, dt):
        dist_mat = self.distances_matrix()
        for boid in range(self.num_boids):
            distances = dist_mat[boid]
            self.centering_force(boid, distances)
            self.boid_avoid_force(boid, distances)
            self.obstacle_avoid_force(boid)
            self.velocity_match_force(boid, distances)
            self.speed_limiter(boid)
        self.move_boids(dt)
