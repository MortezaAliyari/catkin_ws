#!/usr/bin/python3
from math import sin, cos, atan2, hypot, exp, floor
from localization.geometry import to_index, to_world, to_grid

def distance(x0, y0, x1, y1): 
    return hypot(x1 - x0, y1 - y0)


def checkInBounds(cell, the_map):
    if cell[0] < 0 or cell[0] > the_map.info.width - 1 or cell[1] < 0 or cell[1] > the_map.info.height - 1:
        return False
    return True


def line_seg(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x = x0
    y = y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2
    while n > 0:
        points.append((x, y))
        if error >= 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
        n -= 1

    return points 


def ray_tracing(x0, y0, angle, the_map): 
    map_max_range = distance(0.0, 0.0, the_map.info.width, the_map.info.height) 
    x1 = x0 + the_map.info.resolution / 2 + map_max_range * cos(angle)
    y1 = y0 + the_map.info.resolution / 2 + map_max_range * sin(angle)
    ray_points = line_seg(int(floor(x0)), int(floor(y0)), int(floor(x1)), int(floor(y1))) 
    for cell in ray_points:
        if not checkInBounds(cell, the_map):
            return
        cellIndex = to_index(cell[0], cell[1], the_map.info.width)
        if the_map.data[cellIndex] == 100:
            return cell 

    return


def expected_scan(x, y, theta, min_angle, increment, n_readings, max_range, the_map): #
    ranges = []
    for rayId in range(n_readings):
        halfCell = the_map.info.resolution /2  
        angle = theta + min_angle + rayId * increment  
        endpoint = ray_tracing(x, y, angle, the_map) 
        if endpoint is None:  
            ranges.append(max_range)
            continue
        ranges.append(distance(x, y, endpoint[0], endpoint[1]) * the_map.info.resolution) 

    return ranges 


def scan_similarity(ranges0, ranges1, max_range): 
    score = 0
    for rayId in range(len(ranges0)):
        absDiff = abs(ranges0[rayId] - ranges1[rayId])
        score += absDiff

    score = 1 - score / max_range / len(ranges0)
    return score 
