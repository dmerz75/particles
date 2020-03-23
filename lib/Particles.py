import os
import sys
import numpy as np
from lib.Motion import *
import matplotlib.pyplot as plt
my_library = os.path.normpath(os.path.expanduser('~/.myconfigs/'))
sys.path.append(my_library)
part_dir = os.path.normpath(os.path.expanduser('~/git/particles/fig'))
linux_dir = os.path.normpath("/mnt/c/Users/merz.d/.myconfigs/")
sys.path.append(linux_dir)

# print(sys.path)
# sys.exit()
from plot.SETTINGS import *

my_dir = os.path.abspath(os.path.dirname('__file__'))


class Particle:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

        self.radius = 0.5

        # state: untouched(0), sick/contagious(1), non-transmissible(-1)
        self.state = 0

        # velocites: range 0.5 to 1.5
        radian = np.random.uniform(0, np.pi*2)
        self.vx = np.cos(radian)
        self.vy = np.sin(radian)

    def assign_properties(self, pos_state):
        self.x = pos_state[0]
        self.y = pos_state[1]

    def print_properties(self):
        print(self.x, self.y, self.vx, self.vy, "R: ", self.radius, "I: ", self.state)



class Configuration:
    def __init__(self, left_bound=10.0, right_bound=10.0):
        self.min_x = int(left_bound)
        # self.min_x = int(0)
        self.min_y = int(left_bound)
        # self.min_y = int(0)

        self.max_x = int(right_bound)
        self.max_y = int(right_bound)

        self.grid = np.meshgrid
        # self.X = np.array
        # self.Y = np.array

    def get_farthest_reaching_particles(self, particles):
        x_large = max([p.x for p in particles])
        y_large = max([p.y for p in particles])
        x_small = min([p.x for p in particles])
        y_small = min([p.y for p in particles])
        print("-------")
        print("origin: ", x_small, y_small)
        print("low right corner: ", x_large, y_small)
        print("upper right: ", x_large, y_large)
        print("upper left: ", x_small, y_large)
        print("-------")

    def print_size(self):
        print("Your universe is:")
        print("X spans: {} {}".format(self.min_x, self.max_x))
        print("Y spans: {} {}".format(self.min_y, self.max_y))
        print(self.grid)

    def construct_grid(self):
        num_x = self.max_x - self.min_x - 1
        num_y = self.max_y - self.min_y - 1
        nx, ny = (num_x, num_y)

        self.x_positions = np.linspace(self.min_x + 1, self.max_x - 1, num_x)
        self.y_positions = np.linspace(self.min_y + 1, self.max_y - 1, num_y)

        self.x_zeros = np.zeros(num_x)
        self.y_zeros = np.zeros(num_y)

        # print(self.x_positions)
        # print(self.y_positions)

        # self.grid_i, self.grid_j = np.meshgrid(self.x_positions, self.y_positions, sparse=False, indexing='ij')
        # self.grid_oi, self.grid_oj = np.meshgrid(self.x_zeros, self.y_zeros, sparse=False, indexing='ij')

        self.grid = [(x, y, 0) for x in range(1, num_x) for y in range(1, num_y)]
        # print(self.grid)
        # self.zeros = np.zeros((num_x, num_y), dtype=[self.grid])


    def display_grid(self, grid_type="occupied"):
        print("displaying grid:")
        if grid_type == "occupied":
            print("occupied:")
            print(self.grid)
            # print(self.zeros)
            # print(self.grid_oi)
            # print(self.grid_oj)
            # print(self.grid_occupy[0])
            # print(self.x_positions)
            # print(self.y_positions)

            # x = self.grid_occupy[0]
            # y = self.grid_occupy[0][0]
            # print(x, len(x))
            # print(y, "y")
        # else:
        #     print("coordinates:")
        #     print(self.grid[0])

    def assign_random_integer_position(self, particles):
        """
        :param particles: Send in the particles.
        :return: grid_occupy
        """
        # array:
        # print(len(particles))
        self.positions = []

        # print(type(self.grid))
        while len(self.positions) < len(particles):
            n = len(self.positions)
            i = np.random.choice(range(len(self.grid)))
            p = self.grid[i]

            # sys.exit()

            if p[2] == 0:
                # print(p, p[0], p[1], p[2])
                self.positions.append((p[0], p[1]))
                self.grid[i] = (p[0], p[1], 1)
                particles[n].x = p[0]
                particles[n].y = p[1]
            else:
                continue


        return particles
    # # Assign positions to particles:
    # for i in range(len(universe.positions)):
    #     x, y = universe.positions[i]
    #     particles[i].x = x
    #     particles[i].y = y


    def plot_particles(self, particles, seed, ts=0):

        fig = plt.figure()
        colors = ["blue", "red"]

        s = particles[0].radius * 15000 / float(self.max_x)

        for p in particles:
            plt.scatter(p.x, p.y, c=colors[p.state], s=s)

        plt.ylim(self.min_y, self.max_y)
        plt.xlim(self.min_x, self.max_x)

        local_dir = os.path.join(part_dir, str(seed))
        P = SaveFig(local_dir, 'config_%s_%s' % (str(seed), str(ts).zfill(6)), dpi=100)
        fig.canvas.flush_events()
        plt.close()


    def propagate_particles(self, particles, ts):

        # print(type(particles))
        # print(particles[0])
        for p in particles:
            p.x = p.x + p.vx * ts
            p.y = p.y + p.vy * ts

        return particles

    def check_for_walls(self, particles):

        for p in particles:

            if (p.x < self.min_x) | (p.x > self.max_x):
                p.vx = p.vx * -1

            if (p.y < self.min_y) | (p.y > self.max_y):
                p.vy = p.vy * -1

        return particles

    def check_for_collisions(self, pa):

        # for p in pa[0:1]:
        #     print(t, p.x, p.y, p.vx, p.vy)

        for i in range(len(pa)):
            for j in range(len(pa)):

                if i <= j:
                    continue

                dist = np.sqrt((pa[j].x - pa[i].x)**2 + (pa[j].y - pa[i].y)**2)

                if dist <= pa[i].radius * 2:
                    # print(dist)
                    # print("collision")
                    # print("components: ")

                    #               1      2
                    # v12 = 2 - 1
                    # x1, v1
                    # pa[i].print_properties()
                    # pa[j].print_properties()

                    # v1 / v21
                    x1, y1 = vector_pos(pa[j], pa[i])
                    vx1, vy1 = vector_vel(pa[j], pa[i])
                    # v2 / v12 / v(2-1) /
                    x2, y2 = vector_pos(pa[i], pa[j])
                    vx2, vy2 = vector_vel(pa[i], pa[j])

                    dp1 = dot_product(vx1, vy1, x1, y1)
                    dp2 = dot_product(vx2, vy2, x2, y2)

                    # magnitude squared
                    m1 = magnitude(x1, y1) ** 2
                    m2 = magnitude(x2, y2) ** 2

                    # print("i:")
                    # print_vector(pa[i].vx, pa[i].vy)
                    pa[i].vx = pa[i].vx - (dp1 / m1) * x1
                    pa[i].vy = pa[i].vy - (dp1 / m1) * y1
                    # print_vector(pa[i].vx, pa[i].vy)

                    # print("j:")
                    # print_vector(pa[j].vx, pa[j].vy)
                    pa[j].vx = pa[j].vx - (dp2 / m2) * x2
                    pa[j].vy = pa[j].vy - (dp2 / m2) * y2
                    # print_vector(pa[j].vx, pa[j].vy)

                    if (pa[j].state == 1) | (pa[i].state == 1):
                        pa[j].state = 1
                        pa[i].state = 1

                    # print_vector(x12, y12)
                    # print_vector(x21, y21)
                    # print_vector(vx12, vy12)
                    # print_vector(vx21, vy21)

                    # sys.exit()
        return pa

    def check_for_red(self, particles):

        digits = [p.state for p in particles]

        if 0 not in digits:
            print("everyone is now sick.")
            sys.exit()
