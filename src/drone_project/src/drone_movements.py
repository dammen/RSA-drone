import math

class DroneMovement(object):
    MAX_SPEED = 0.6
    def __init__(self, p, w, h):
        self.pattern = p
        self.width = w
        self.height = h
        self.center = (w / 2, h / 2)

    def make_decision(self):
        x, y = self.xy_distance(self.center, (self.pattern.x, self.pattern.y))
        return (self.normalise_width(x), self.normalise_height(y), 0)

    def distance(self, a, b):
        return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * a[1] - b[1])

    def xy_distance(self, fr, to):
        x = to[0] - fr[0]
        y = fr[1] - to[1]
        return (x, y)

    def normalise_width(self, x):
        percentage = float(x) / (self.width / 2)
        return self.MAX_SPEED * percentage

    def normalise_height(self, y):
        percentage = float(y) / (self.height / 2)
        return self.MAX_SPEED * percentage