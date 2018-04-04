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

        self.drawCallback = None

        self.graph = {}
        for node in graph:
            self.graph[(node.x, node.y)] = node

    def heuristic(self, node):
        return math.sqrt((self.goal.x - node.x)**2 + (self.goal.y - node.y)**2)

    def compute(self, draw):
        frontier = PriorityQueue()
        frontier.put((0, self.start))

        frontier_nodes = [self.start]

        came_from = {}
        came_from[self.start] = None

        cost_so_far = {}
        cost_so_far[self.start] = 0

        while not frontier.empty():
            queue_entry = frontier.get()
            current = queue_entry[1]
            frontier_nodes.remove(current)

            for next_node in current.neighbors:
                 # Get the current node from the graph
                next_node = self.graph[next_node]
                new_cost = cost_so_far[current] + 1 # TODO add graph cost

                if (next_node not in cost_so_far) or (new_cost < cost_so_far[next_node]):
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node)
                    frontier.put((priority, next_node))
                    frontier_nodes.append(next_node)
                    came_from[next_node] = current

            if (draw):
                # Update frontier visualization
                self.drawCallback(frontier_nodes, "frontier")

                # Update already explored visualization
                already_explored = []
                for node in cost_so_far:
                    if (node not in frontier_nodes):
                        already_explored.append(node)
                self.drawCallback(already_explored, "explored")

            if current == self.goal:
                break


        # Work our way back through the came_from chain from the goal to the start
        path = [self.goal]
        previousNode = came_from[self.goal]
        while (previousNode != None):
            path.insert(0, previousNode)
            previousNode = came_from[previousNode]

        return path
