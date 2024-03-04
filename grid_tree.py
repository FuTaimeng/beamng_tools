import numpy as np

class GridNode:
    def __init__(self, center, length):
        self.cnt = 0
        self.center = center
        self.length = length

    def contains(self, x, y):
        cx, cy = self.center
        l = self.length / 2
        return x>=cx-l and x<cx+l and y>=cy-l and y<cy+l

class GridTree:
    def __init__(self):
        self.root = None
        self.leaf_length = 0.1

    def add_point(self, x, y, data):
        if self.root is None or not self.root.contains(x, y):
            pass
