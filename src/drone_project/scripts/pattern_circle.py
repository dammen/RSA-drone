'''
Circle class for patterns
Ben Soh
'''
import math


class PatternCircle(object):
    def __init__(self, x, y, c):
        self.colour = c
        self.x = x
        self.y = y

    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)