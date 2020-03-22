from lib.Particles import *


def vector_pos(p1, p2):
    x = p2.x - p1.x
    y = p2.y - p1.y
    return x, y


def vector_vel(p1, p2):
    x = p2.vx - p1.vx
    y = p2.vy - p1.vy
    return x, y


def print_vector(p1, p2):
    print("vector: ", p1, p2)


def dot_product(x1, x2, y1, y2):
    return x1 * y1 + x2 * y2


def magnitude(x1, y1):
    return np.sqrt(x1 ** 2 + y1 ** 2)


def scalar_multiply_pos(x, v):
    return x * v.vx, x * v.vy


def normalize(x, y):
    pass