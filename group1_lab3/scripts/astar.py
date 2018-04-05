#!/usr/bin/env python
import rospy, tf, copy, math
from Queue import PriorityQueue

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, GridCells, Path

from tf.transformations import quaternion_from_euler
import numpy as np
from std_msgs.msg import String

import group1_lab3.srv

class Node:
    def __init__(self, x, y, neighbors):
        self.x = x
        self.y = y
        self.neighbors = neighbors # List of (x, y) tuples

class AStar:
    def __init__(self):
        self.draw = True

        self._map = OccupancyGrid()

        self._grid_path_pub = rospy.Publisher('/astar_grid_path', GridCells, latch=True, queue_size=1)
        self._grid_frontier_pub = rospy.Publisher('/astar_grid_frontier', GridCells, latch=True, queue_size=10)
        self._grid_explored_pub = rospy.Publisher('/astar_grid_explored', GridCells, latch=True, queue_size=10)

        rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1)

    def updateMap(self, new_map):
        self._map = new_map

        # Parse map and create graph
        self.graph = {}

        for x in range(0, self._map.info.width):
            for y in range(0, self._map.info.height):
                if (self._map.data[y * self._map.info.height + x] == 0):
                    neighbors = self.getNeighbors(x, y)
                    node = Node(x, y, neighbors)
                    self.graph[(x, y)] = node

    def getNeighbors(self, x, y):
        neighbors = []
        #for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if (xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height): continue

            if (self._map.data[yc * self._map.info.height + xc] == 0):
                neighbors.append(coord)

        return neighbors

    def heuristic(self, node, goal):
        return math.sqrt((goal.x - node.x)**2 + (goal.y - node.y)**2)

    def handle_request(self, req):
        start_x = int(req.start_x / self._map.info.resolution)
        start_y = int(req.start_y / self._map.info.resolution)

        goal_x = int(req.goal_x / self._map.info.resolution)
        goal_y = int(req.goal_y / self._map.info.resolution)

        start = self.graph[(start_x, start_y)]
        goal  = self.graph[(goal_x, goal_y)]

        self.compute(start, goal)

    def compute(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))

        frontier_nodes = [start]

        came_from = {}
        came_from[start] = None

        cost_so_far = {}
        cost_so_far[start] = 0

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
                    priority = new_cost + self.heuristic(next_node, goal)
                    frontier.put((priority, next_node))
                    frontier_nodes.append(next_node)
                    came_from[next_node] = current

            if (self.draw):
                # Update frontier visualization
                self.drawNodes(frontier_nodes, "frontier")

                # Update already explored visualization
                already_explored = []
                for node in cost_so_far:
                    if (node not in frontier_nodes):
                        already_explored.append(node)
                self.drawNodes(already_explored, "explored")

            if current == goal:
                break

        # Work our way back through the came_from chain from the goal to the start
        path = [goal]
        previousNode = came_from[goal]
        while (previousNode != None):
            path.insert(0, previousNode)
            previousNode = came_from[previousNode]

        # Compute ros Path
        waypoints = []
        index = 0
        current_dir = None
        for node in path:
            try:
                next_node = path[index + 1]

                # If we've changed directions, and add a waypoint if so
                this_dir = math.atan2((next_node.y - node.y), (next_node.x - node.x))
                if (current_dir == None or this_dir != current_dir):
                    current_dir = this_dir

                    # copied some stuff from
                    # https://www.programcreek.com/python/example/70252/geometry_msgs.msg.PoseStamped
                    pose = PoseStamped()
                    pose.header.seq = len(waypoints)
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()

                    pose.pose.position.x = (node.x * self._map.info.resolution) + (self._map.info.resolution / 2)
                    pose.pose.position.y = (node.y * self._map.info.resolution) + (self._map.info.resolution / 2)
                    pose.pose.position.z = 0

                    q = quaternion_from_euler(0.0, 0.0, current_dir)
                    pose.pose.orientation = Quaternion(*q)

                    waypoints.append(pose)

                index += 1
            except IndexError:
                # We've reached the end of the loop, add last waypoint
                pose = PoseStamped()
                pose.header.seq = len(waypoints)
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = (node.x * self._map.info.resolution) + (self._map.info.resolution / 2)
                pose.pose.position.y = (node.y * self._map.info.resolution) + (self._map.info.resolution / 2)
                pose.pose.position.z = 0

                q = quaternion_from_euler(0.0, 0.0, current_dir)
                pose.pose.orientation = Quaternion(*q)

                waypoints.append(pose)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = waypoints

        return path_msg

    def drawNodes(self, nodes, topic):
        publisher = None
        if (topic == "frontier"):
            publisher = self._grid_frontier_pub
        elif (topic == "explored"):
            publisher = self._grid_explored_pub
        elif (topic == "path"):
            publisher = self._grid_path_pub
        else:
            return

        cells = GridCells()
        cells.header.frame_id = self._map.header.frame_id
        cells.cell_width = self._map.info.resolution
        cells.cell_height = self._map.info.resolution

        cellPoints = []
        for node in nodes:
            cellX = (node.x * self._map.info.resolution) + (self._map.info.resolution / 2)
            cellY = (node.y * self._map.info.resolution) + (self._map.info.resolution / 2)
            cellPoints.append(Point(cellX, cellY, 0))

        cells.cells = cellPoints
        publisher.publish(cells)

if __name__ == "__main__":
    astar = AStar()

    rospy.init_node('astar_server')
    s = rospy.Service('astar', group1_lab3.srv.AStar, astar.handle_request)

    rospy.spin()
