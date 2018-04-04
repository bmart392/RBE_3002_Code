from geometry_msgs.msg import Point
import math
from Queue import PriorityQueue

class Node:
    def __init__(self, x, y, neighbors):
        self.x = x
        self.y = y
        self.neighbors = neighbors # List of (x, y) tuples

class AStar:
    def __init__(self, graph, start, goal):
        self.start = start
        self.goal = goal

        self.graph = {}
        for node in graph:
            self.graph[(node.x, node.y)] = node

    def heuristic(self, node):
        return math.sqrt((self.goal.x - node.x)**2 + (self.goal.y - node.y)**2)

    def compute(self):
        frontier = PriorityQueue()
        frontier.put(self.start, 0)

        came_from = {}
        came_from[self.start] = None

        cost_so_far = {}
        cost_so_far[self.start] = 0

        while not frontier.empty():
            current = frontier.get()

            #if current == self.goal:
            #    break

            for next in current.neighbors:
                 # Get the current node from the graph
                next = self.graph[next]
                new_cost = cost_so_far[current] + 1 # TODO add graph cost

                if (next not in cost_so_far) or (new_cost < cost_so_far[next]):
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next)
                    frontier.put(next, priority)
                    came_from[next] = current

        # Work our way back through the came_from chain from the goal to the start
        path = [self.goal]
        previousNode = came_from[self.goal]
        while (previousNode != None):
            path.insert(0, previousNode)
            previousNode = came_from[previousNode]

        return path
