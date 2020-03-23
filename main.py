#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

my_dir = os.path.abspath(os.path.dirname('__file__'))
linux_dir = "/mnt/c/Users/merz.d/git/particles"
sys.path.append(my_dir)
sys.path.append(linux_dir)

from lib.Particles import *


def main():
    print("Begin particles!")

    num_particles = 25
    time_step = 0.005
    total_steps = 150000
    X = 0.0
    Y = 20.0
    seed = 1000654
    np.random.seed(seed)
    print("seed: ", seed)

    # Create Particles:
    particles = [Particle() for x in range(num_particles)]

    # Infected Particle:
    particles[0].state = 1

    # for p in particles:
    #     print(p, p.x, p.y)

    # Create Universe:
    universe = Configuration(X, Y)
    universe.print_size()
    universe.construct_grid()

    # Get grid positions for particles:
    # universe.display_grid("occupied")
    particles = universe.assign_random_integer_position(particles)
    # universe.display_grid("occupied")
    print("Particles positioned: ", len(universe.positions))
    print(particles[0].x, particles[0].y, particles[0].vx, particles[0].vy)
    print(type(particles))
    # Assign positions to particles:
    # for i in range(len(universe.positions)):
    #     x, y = universe.positions[i]
    #     particles[i].x = x
    #     particles[i].y = y

    # Check velocites:
    # for i in range(len(particles)):
    #     particles[i].print_properties()

    # See starting configuration:
    universe.plot_particles(particles, seed)

    # Propagate: x = vt, dx/dt * t
    for step in range(1, total_steps):

        # # move
        # for p in particles:
        #     p.x = p.x + p.vx * time_step
        #     p.y = p.y + p.vy * time_step

        # move
        # print(particles[0])
        particles = universe.propagate_particles(particles, time_step)
        # check for walls:
        particles = universe.check_for_walls(particles)
        # check collisions
        # print("Step: ", step)
        particles = universe.check_for_collisions(particles)
        # check if all red!
        universe.check_for_red(particles)

        if step % 100 == 0:
            # print(step)
            universe.plot_particles(particles, seed, step)


if __name__ == '__main__':
    main()
