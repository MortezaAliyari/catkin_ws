#!/usr/bin/python
from math import sin, cos, atan2, hypot, exp, floor
from geometry import to_index, to_world, to_grid

def distance(x0, y0, x1, y1): #利用歐幾里得距離的套件來計算兩點之間的距離
    return hypot(x1 - x0, y1 - y0)


def checkInBounds(cell, the_map):#確認點是否在地圖範圍內
    if cell[0] < 0 or cell[0] > the_map.info.width - 1 or cell[1] < 0 or cell[1] > the_map.info.height - 1:
        return False
    return True


def line_seg(x0, y0, x1, y1):#起始點(x0,y0)與地圖邊界點(x1,y1)
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

    return points #是一個陣列，代表原點與地圖邊界上的點路經之間所有被偵測出來的點


def ray_tracing(x0, y0, angle, the_map): #計算雷達掃描到障礙物的座標
    map_max_range = distance(0.0, 0.0, the_map.info.width, the_map.info.height) #計算地圖的斜邊(地圖的最左下角與最右上角的座標)
    x1 = x0 + the_map.info.resolution / 2 + map_max_range * cos(angle)#計算邊界點的座標
    y1 = y0 + the_map.info.resolution / 2 + map_max_range * sin(angle)#計算邊界點的座標   map_max_range:定義成地圖斜邊長度
    ray_points = line_seg(int(floor(x0)), int(floor(y0)), int(floor(x1)), int(floor(y1))) #將地圖上起始點與邊界點之間的點全部掃進來
    for cell in ray_points:
        if not checkInBounds(cell, the_map):#判斷點是否在地圖邊界內，如果超出地圖邊界則回傳none，反之，將該點座標從二維轉一維
            return
        cellIndex = to_index(cell[0], cell[1], the_map.info.width)#將二維座標轉成一維座標
        if the_map.data[cellIndex] == 100:#判斷點是否為黑色，黑色的點定義成障礙物
            return cell #回傳這些障礙物的座標

    return


def expected_scan(x, y, theta, min_angle, increment, n_readings, max_range, the_map): #
    ranges = []
    for rayId in range(n_readings):
        halfCell = the_map.info.resolution /2  #取得地圖一半的解析度
        angle = theta + min_angle + rayId * increment  #將0~360度分成90等份，掃描路徑會有90個角度
        endpoint = ray_tracing(x, y, angle, the_map) #此函式是在計算起始點90個角度的掃描路徑上障礙物的座標
        if endpoint is None:  #如果掃描路徑上沒有障礙物，則回傳max_range
            ranges.append(max_range)
            continue
        ranges.append(distance(x, y, endpoint[0], endpoint[1]) * the_map.info.resolution) #如果偵測到障礙物，則把障礙物的座標與起始點之間的距離算出來

    return ranges #回傳所有距離


def scan_similarity(ranges0, ranges1, max_range): #計算相似度
    score = 0
    for rayId in range(len(ranges0)):
        absDiff = abs(ranges0[rayId] - ranges1[rayId])
        score += absDiff

    score = 1 - score / max_range / len(ranges0)
    return score ** 2