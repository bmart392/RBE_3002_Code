from geometry_msgs.msg import Point

class Node:
    def __init(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost

class AStar:

    def __init__(self, grid, start, target):
        self.grid = grid
        self.start = start
        self.target = target

    def compute(self):
        knownNodes = []
        discoveredNodes = []
        path = []
        costs = {}
        costs[self.start] = 0
