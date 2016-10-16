'''
Math utility methods
Ben Soh
'''
import math


def rotate_point(point, rad):
    x, y = point
    new_x = x * math.cos(rad) - y * math.sin(rad)
    new_y = y * math.cos(rad) + x * math.sin(rad)
    return (int(new_x), int(new_y))

def line_angle(a, b):
    x1, y1 = a
    x2, y2 = b
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
    if x1 == x2:
        return 90
    gradient = float(y2 - y1) / (x2 - x1)
    return math.atan(gradient)