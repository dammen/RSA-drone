#!/usr/bin/env python
'''
Pattern class
Ben Soh
'''

class Pattern(object):
    def __init__(self, x, y, ID, rotation, circles=[]):
        self.x = x
        self.y = y
        self.ID = ID
        self.rotation = rotation
        self.circles = circles
