from geometry import *
from math import pi
import random, matplotlib.pyplot as plt, matplotlib.patches as patches

def random_particle(the_map):
    foundFreeCell = False
    while not foundFreeCell: #判斷取樣點是否在障礙物上，如果採樣點沒有障礙物則跳出迴圈，反之則重新採樣
        x = random.uniform(0, the_map.info.width)
        y = random.uniform(0, the_map.info.height)
        cellId = to_index(int(x), int(y), the_map.info.width)
        if the_map.data[cellId] == 0:
            foundFreeCell = True

    x = the_map.info.origin.position.x + x * the_map.info.resolution#將取樣點的x座標與y座標進行座標轉換
    y = the_map.info.origin.position.y + y * the_map.info.resolution
    theta = random.uniform(-pi, +pi) #theta 為隨機採樣一個角度
    return (x, y, theta) 


def new_particle(particle, spatial_var, angle_var): #將權重較低的粒子xy座標進行重新採樣(利用現有的資料重新做處理,使用高斯分布隨機產生數字並返回具有高斯分布的隨機浮點數)
    x = random.gauss(particle[0], spatial_var)
    y = random.gauss(particle[1], spatial_var)
    theta = random.gauss(particle[1], angle_var)
    return (x, y, theta) #


def resample(particles_weighted, n_particles): #重新採樣的函式
    weightSum = 0 #一開始先定義總權重
    for particle in particles_weighted: #將相似度全部相加即為總權重
        weightSum += particle[0]

    r = random.uniform(0, 1 / float(n_particles)) #定義有效的粒子數，假設相似度分數太低的粒子則不考慮進來，避免過多無效的運算
    i = 0
    w = particles_weighted[0][0] / weightSum #第一個有效粒子去除以總權重
    particles = []
    for m in range(n_particles): # n_partical為一開始定義的隨機採樣數
        U = r + float(m) / float(n_particles) # 判斷權重是否有高過門檻，U是設定門檻，如果高過門檻就會重新採樣
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