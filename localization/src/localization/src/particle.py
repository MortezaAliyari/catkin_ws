from localization.geometry import *
from math import pi
import random, matplotlib.pyplot as plt, matplotlib.patches as patches

def random_particle(the_map):
    foundFreeCell = False
    while not foundFreeCell:
        x = random.uniform(0, the_map.info.width)
        y = random.uniform(0, the_map.info.height)
        cellId = to_index(int(x), int(y), the_map.info.width)
        if the_map.data[cellId] == 0:
            foundFreeCell = True

    x = the_map.info.origin.position.x + x * the_map.info.resolution
    y = the_map.info.origin.position.y + y * the_map.info.resolution
    theta = random.uniform(-pi, +pi) 
    return (x, y, theta) 


def new_particle(particle, spatial_var, angle_var): 
    x = random.gauss(particle[0], spatial_var)
    y = random.gauss(particle[1], spatial_var)
    theta = random.gauss(particle[1], angle_var)
    return (x, y, theta) #


def resample(particles_weighted, n_particles): 
    weightSum = 0 
    for particle in particles_weighted:
        weightSum += particle[0]

    r = random.uniform(0, 1 / float(n_particles)) 
    i = 0
    w = particles_weighted[0][0] / weightSum 
    particles = []
    for m in range(n_particles): 
        U = r + float(m) / float(n_particles) 
        while U > w:
            i += 1
            w += particles_weighted[i][0] / weightSum

        particle = particles_weighted[i][1]
        particles.append(new_particle(particle, 0.05, pi / 8))

    return particles


def draw_occupancy_grid(the_map, ax):
    for cellId in range(len(the_map.data)):
        x = cellId // the_map.info.width
        y = cellId % the_map.info.width
        x, y = to_world(x, y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
        res = the_map.info.resolution
        if the_map.data[cellId] == 100:
            patch = patches.Rectangle((x - res / 2, y - res / 2), res, res, color='k', alpha=0.5)
            ax.add_patch(patch)
        elif the_map.data[cellId] == 0:
            patch = patches.Rectangle((x - res / 2, y - res / 2), res, res, color='b', alpha=0.5)
            ax.add_patch(patch)

    None
    return


def draw_particles_scored(particles_weighted):
    scoresAssigned = False
    for particle in particles_weighted:
        if particle[0] != 0.0:
            scoresAssigned = True

    for ptclId in range(len(particles_weighted)):
        x, y, theta = particles_weighted[ptclId][1]
        mSize = 0.1
        if scoresAssigned:
            mSize = particles_weighted[ptclId][0] * 20
        plt.plot([x, x - math.sin(theta) * 0.5], [
         y, y + math.cos(theta) * 0.5], 'g', linewidth=mSize / 5)
        plt.plot(x, y, 'ro', markersize=mSize, markeredgecolor='r')

    None
    return


def debug_call(particles_weighted, the_map):
    debug = False
    if not debug:
        return
    my_dpi = 96
    plt.figure(1, figsize=(800 / my_dpi, 800 / my_dpi), dpi=my_dpi)
    plt.cla()
    plt.xlim(the_map.info.origin.position.x, the_map.info.origin.position.x + the_map.info.width)
    plt.ylim(the_map.info.origin.position.y, the_map.info.origin.position.y + the_map.info.height)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X world')
    plt.xlabel('Y world')
    ax = plt.axes()
    draw_occupancy_grid(the_map, ax)
    draw_particles_scored(particles_weighted)
    plt.draw()
    pause = True
    if pause:
        k = plt.waitforbuttonpress(1)
        while not k:
            k = plt.waitforbuttonpress(1)

    else:
        plt.waitforbuttonpress(1e-06)
