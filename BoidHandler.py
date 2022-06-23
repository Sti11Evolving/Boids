import numpy as np
from numpy import linalg as la
import random


# Probably could be more readable if boids were individual objects, but using numpy arrays for performance's sake
# Holds positional, velocity, and color for all boids in 2D numpy array and updating functions
class BoidHandler:
    def __init__(self, environment_width, environment_height, max_speed=2):
        self.environment_width = environment_width
        self.environment_height = environment_height

        self._boids_pos = np.empty((0, 2), dtype=int)  # 2D array holding boid positional data
        # First dimension is for each boid
        # Second dimension is x or y
        # i.e. boids_pos[0][1] would get the y position for the first boid

        self._boids_vel = np.empty((0, 2), dtype=np.single)  # 2D array holding boid velocity data
        # First dimension is for each boid
        # Second dimension is x_vel or y_vel
        # i.e. boids_pos[1][0] would get the x_vel position for the second boid

        self._boids_color = np.empty((0, 3), dtype=int)  # 2D array holding boid color data
        self.boid_nums = 0
        self.dist_mat = None

        # Constants that determine the boids flocking behavior
        self.max_speed = max_speed
        self.desired_speed = max_speed/2
        self.min_speed = max_speed/4
        self.centering_factor = 0.000009
        self.min_distance = 50
        self.avoid_factor = 0.1
        self.obstacle_avoid_factor = 0.1
        self.group_range = 100
        self.obstacle_range = 50
        self.matching_force = .009
        self.speed_mod = 0.1

    # Reset all boid data
    def clear_boids(self):
        self._boids_pos = np.empty((0, 2), dtype=int)
        self._boids_vel = np.empty((0, 2), dtype=np.single)
        self._boids_color = np.empty((0, 3), dtype=int)
        self.boid_nums = 0

    # Add boid data for a new boid to the data list
    def add_boid(self, pos=[[0., 0.]], vel=[[0., 0.]], color=[[255, 0, 0]]):
        self._boids_pos = np.concatenate((self._boids_pos, pos), dtype=np.single, axis=0)
        self._boids_vel = np.concatenate((self._boids_vel, vel), dtype=np.single, axis=0)
        self._boids_color = np.concatenate((self._boids_color, color), axis=0)
        self.boid_nums += 1

    # returns a dictionary containing information for a particular boid for debug purposes
    def get_boid(self, boid_num):
        return {
            "x": self._boids_pos[boid_num][0],
            "y": self._boids_pos[boid_num][1],
            "x_vel": self._boids_vel[boid_num][0],
            "y_vel": self._boids_vel[boid_num][1],
            "color": self._boids_color[boid_num],
        }

    def get_boids(self):
        return list(map(lambda boid: self.get_boid(boid), range(self.boid_nums)))

    def add_random_boids(self, boid_nums, env_width, env_height):
        random.seed()
        start_speed_bound = self.max_speed/2
        for boid in range(boid_nums):
            self.add_boid([[random.randrange(0, env_width), random.randrange(0, env_height)]],
                          [[random.uniform(start_speed_bound*-1, start_speed_bound),
                            random.uniform(start_speed_bound*-1, start_speed_bound)]])

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
        dist_mat = np.zeros(shape=[self.boid_nums, self.boid_nums], dtype=int)
        for boid in range(self.boid_nums - 1):
            dist = self.get_distances(boid, range(boid, self.boid_nums))
            dist_mat[boid::, boid] = dist
            dist_mat[boid][boid::] = dist
        return dist_mat

    # Returns an array of boid nums in a particular radius from a particular boid
    # Result will include the boid that distances are calculated from
    def _get_nearby(self, boid_num, radius):
        return list(filter(lambda boid: self.dist_mat[boid_num][boid] <= radius and boid != boid_num,
                           range(self.boid_nums)))

    # Calculate force of boids to turn towards center of group
    def _centering_force(self, dt):
        for boid_num in range(self.boid_nums):
            group = self._get_nearby(boid_num, self.group_range)
            if len(group):
                center = np.zeros(2, dtype=np.single)
                for neighbor in group:
                    center += self._boids_pos[neighbor]
                center = center / len(group)
                self._boids_vel[boid_num] += ((center - self._boids_pos[boid_num]) * self.centering_factor) ** len(group) * dt

    # Calculate object avoiding force
    def _edge_avoidance(self, dt):
        for boid_num in range(self.boid_nums):
            margin = self.obstacle_range
    
            boid_x = self._boids_pos[boid_num][0]
            if boid_x < margin:
                if boid_x < 0:
                    self._boids_pos[boid_num][0] = 0
                proximity_mod = ((margin - boid_x)/margin)**2
                self._boids_vel[boid_num][0] += proximity_mod * self.obstacle_avoid_factor * dt
            elif boid_x > self.environment_width - margin:
                if boid_x > self.environment_width:
                    self._boids_pos[boid_num][0] = self.environment_width
                proximity_mod = (((self.environment_width - margin) - boid_x)/margin)**2
                self._boids_vel[boid_num][0] -= proximity_mod * self.obstacle_avoid_factor * dt
    
            boid_y = self._boids_pos[boid_num][1]
            if boid_y < margin:
                if boid_y < 0:
                    self._boids_pos[boid_num][1] = 0
                proximity_mod = ((margin - boid_y)/margin)**2
                self._boids_vel[boid_num][1] += proximity_mod * self.obstacle_avoid_factor * dt
            elif boid_y > self.environment_height - margin:
                if boid_y > self.environment_height:
                    self._boids_pos[boid_num][1] = self.environment_height
                proximity_mod = (((self.environment_height - margin) - boid_y)/margin)**2
                self._boids_vel[boid_num][1] -= proximity_mod * self.obstacle_avoid_factor * dt

    # Calculate force avoiding collisions with nearby boids
    def _boid_avoid_force(self, dt):
        for boid_num in range(self.boid_nums):
            boids_to_avoid = self._get_nearby(boid_num, self.min_distance)
            if len(boids_to_avoid):
                avoid_vel = np.zeros(2, dtype=np.single)
                for other_boid in boids_to_avoid:
                    diff = self._boids_pos[boid_num] - self._boids_pos[other_boid]
                    closeness_factor = ((self.min_distance - self.dist_mat[boid_num][other_boid])/self.min_distance)**2
                    avoid_vel += diff * (closeness_factor * self.avoid_factor)**2 * dt
                self._boids_vel[boid_num] += avoid_vel

    # Calculate force matching velocity with neighboring boids
    def _velocity_match_force(self, dt):
        for boid_num in range(self.boid_nums):
            group = self._get_nearby(boid_num, self.group_range)
            if len(group):
                avg_group_vel = np.zeros(2, dtype=np.single)
                for other_boid in group:
                    avg_group_vel += self._boids_vel[other_boid]
                avg_group_vel /= len(group)
                self._boids_vel[boid_num] += (avg_group_vel - self._boids_vel[boid_num]) * self.matching_force

    def _speed_limiter(self):
        for boid_num in range(self.boid_nums):
            speed = la.norm(self._boids_vel[boid_num])
            direction = self._boids_vel[boid_num] / speed
            if speed > self.max_speed:
                self._boids_vel[boid_num] = self.max_speed * direction
            elif speed < self.min_speed:
                self._boids_vel[boid_num] = self.min_speed * direction

    def _desired_speed_force(self, dt):
        for boid_num in range(self.boid_nums):
            speed = la.norm(self._boids_vel[boid_num])
            if speed == 0:
                self._boids_vel[boid_num] += [self.desired_speed/10, self.desired_speed/10]
            else:
                speed_diff = self.desired_speed - speed
                direction = self._boids_vel[boid_num] / speed
                self._boids_vel[boid_num] += direction * speed_diff * .01 * dt

    def _move_boids(self, dt):
        move_amount = np.multiply(self._boids_vel, dt * self.speed_mod)
        self._boids_pos += move_amount

    def update(self, obstacles, dt):
        self.dist_mat = self.distances_matrix()
        self._centering_force(dt)
        self._boid_avoid_force(dt)
        self._edge_avoidance(dt)
        self._desired_speed_force(dt)
        self._speed_limiter()
        self._velocity_match_force(dt)
        self._move_boids(dt)
