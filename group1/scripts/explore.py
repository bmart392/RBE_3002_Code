#!/usr/bin/env python
import rospy, math
from nav_msgs.msg import OccupancyGrid
from astar import Astar, Node
from robot import Robot

class Explore:

    def __init__(self):

        self._map = OccupancyGrid()

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.explore, queue_size=1)

    def explore(self):

            frontier_nodes = []

            for x in range(0, self._map.info.width):
                for y in range(0, self._map.info.height):
                    if self._map.data[y * self._map.info.height + x] > 0:  # IS THIS A WALL? NO? Go on
                        neighbors = self.getNeighbors(x, y)
                        node = Node(x, y, neighbors)
                        for node in neighbors:
                            if self._map.data[node.y * self._map.info.height + node.x] < 0:
                                frontier_nodes.append(Node(x, y, []))

            if not frontier_nodes:
                return

            distance = 100 # Random number that will guarantee the distance is always smaller. Probably need to TODO make better

            for node in frontier_nodes:
                if math.sqrt((x ** 2 - node.x ** 2) + (y ** 2 - node.y ** 2)) < distance:
                    distance = math.sqrt((x ** 2 - node.x ** 2) + (y ** 2 - node.y ** 2))
                    node_to_go = node

                    # node is where we want to go

            frontier_goal = tuple(node_to_go.x, node_to_go.y)

            self.setEndNode(frontier_goal) # Should in theory astar to the frontier and drive there? Maybe?
