#!/usr/bin/env python
import rospy, tf, copy, math
from Queue import PriorityQueue

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate

from tf.transformations import quaternion_from_euler
import numpy as np
from std_msgs.msg import String

import group1.srv


class Node:
    def __init__(self, x, y, neighbors):
        self.x = x
        self.y = y
        self.neighbors = neighbors  # List of (x, y) tuples

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)


class AStar:
    def __init__(self):
        self.draw = True

        self._map = OccupancyGrid()
        self._graph = {}
        self._local_costmap_offset = (0, 0)
        self._costmap = None

        self._grid_path_pub = rospy.Publisher('/astar_grid_path', GridCells, latch=True, queue_size=1)
        self._grid_frontier_pub = rospy.Publisher('/astar_grid_frontier', GridCells, latch=True, queue_size=10)
        self._grid_explored_pub = rospy.Publisher('/astar_grid_explored', GridCells, latch=True, queue_size=10)
        self._grid_wall_pub = rospy.Publisher('/astar_grid_walls', GridCells, latch=True, queue_size=10)

        self._opt_map_pub = rospy.Publisher('/astar_opt_map', OccupancyGrid, latch=True, queue_size=10)
        self._our_costmap = rospy.Publisher('/my_costmap', OccupancyGrid, latch=True, queue_size=10)

        #rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.initGlobalCostmap, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.initGlobalCostmap, queue_size=1)
        #rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.localCostmapMoved, queue_size=1)
        #rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, self.localCostmapUpdate, queue_size=1)

    def updateMap(self, new_map):
        start_time = rospy.Time.now()
        self._map = copy.deepcopy(new_map)

        # Expand map expand_iterations times
        expand_iterations = 1
        for i in range(expand_iterations):
            data = list(new_map.data)
            for x in range(0, self._map.info.width):
                for y in range(0, self._map.info.height):
                    if self._map.data[y * self._map.info.height + x] > 0:
                        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
                            xc = coord[0]
                            yc = coord[1]

                            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                                continue

                            data[yc * self._map.info.height + xc] = 100

            self._map.data = tuple(data)

        # Create optimized map
        opt_map = OccupancyGrid()
        opt_map.header.seq = 0
        opt_map.header.stamp = rospy.Time.now()
        opt_map.header.frame_id = "map"  # TODO correct frame id?

        # Calculate new map grid size
        robot_size = 0.178 * 2
        new_map_size = self._map.info.resolution
        while (new_map_size < self._map.info.resolution / robot_size):
            new_map_size += self._map.info.resolution

        new_width = int(self._map.info.width * (self._map.info.resolution / new_map_size))
        new_height = int(self._map.info.height * (self._map.info.resolution / new_map_size))

        opt_map.info.resolution = new_map_size
        opt_map.info.width = new_width
        opt_map.info.height = new_height
        opt_map.info.origin = self._map.info.origin

        # Fill in new map
        opt_map.data = [0] * (new_width * new_height)
        for x in range(0, self._map.info.width):
            for y in range(0, self._map.info.height):
                if self._map.data[y * self._map.info.height + x] > 0:
                    nx = int((float(x) / self._map.info.width) * new_width)
                    ny = int((float(y) / self._map.info.height) * new_height)
                    index = ny * new_height + nx
                    opt_map.data[index] = 100

        self._map = opt_map

        # Parse map and create graph
        self._graph = {}

        wall_nodes = []
        for x in range(0, self._map.info.width):
            for y in range(0, self._map.info.height):
                if self._map.data[y * self._map.info.height + x] == 0:  # IS THIS A WALL? NO? Go on
                    neighbors = self.getNeighbors(x, y)
                    node = Node(x, y, neighbors)
                    self._graph[(x, y)] = node
                else:
                    wall_nodes.append(Node(x, y, []))

        end_time = rospy.Time.now()
        print ("Update Map Time", end_time - start_time)
        self.drawNodes(wall_nodes, "wall")
        self._opt_map_pub.publish(self._map)

    def initGlobalCostmap(self, costmap):
        print "New global costmap"
        self._costmap = costmap
        self._our_costmap.publish(self._costmap)
        self.updateMap(self._costmap)
        #self.explore()

    # def localCostmapMoved(self, costmap):
    #     # Wait for us to get a global costmap
    #     while (self._costmap is None):
    #         pass
    #
    #     print "Local costmap moved"
    #     offset = costmap.info.origin.position
    #
    #     xc = int((offset.x - self._costmap.info.origin.position.x) / self._costmap.info.resolution)
    #     yc = int((offset.y - self._costmap.info.origin.position.y) / self._costmap.info.resolution)
    #
    #     self._local_costmap_offset = (xc, yc)
    #
    # def localCostmapUpdate(self, costmap):
    #     offset = self._local_costmap_offset
    #
    #     print "Update local costmap"
    #     print offset
    #
    #     new_data = list(self._costmap.data)
    #     start_x = costmap.x + offset[0]
    #     start_y = costmap.y + offset[1]
    #     for x in range(start_x, start_x + costmap.width):
    #         for y in range(start_y, start_y + costmap.height):
    #             (xc, yc) = (x - start_x, y - start_y)
    #             this_data = costmap.data[yc * costmap.height + xc]
    #             new_data[y * self._costmap.info.height + x] = this_data
    #
    #     # Create map update
    #     self._costmap.header.stamp = rospy.Time.now()
    #     self._costmap.data = tuple(new_data)
    #
    #     self._our_costmap.publish(self._costmap)
    #
    #     self.updateMap(self._costmap)

    def mapCoordsToWorld(self, x, y):
        xc = (x * self._map.info.resolution) + (self._map.info.resolution / 2) + (self._map.info.origin.position.x)
        yc = (y * self._map.info.resolution) + (self._map.info.resolution / 2) + (self._map.info.origin.position.y)

        return xc, yc

    def worldCoordToMap(self, x, y):
        xc = int((x - self._map.info.origin.position.x) / self._map.info.resolution)
        yc = int((y - self._map.info.origin.position.y) / self._map.info.resolution)

        return xc, yc

    def getNeighbors(self, x, y):
        neighbors = []
        # for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                continue

            if self._map.data[yc * self._map.info.height + xc] == 0:
                neighbors.append(coord)

        return neighbors

    def heuristic(self, node, goal):
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return dx + dy # math.sqrt(dx**2 + dy**2)

    def handle_request(self, req):
        (start_x, start_y) = self.worldCoordToMap(req.start_x, req.start_y)
        (goal_x, goal_y) = self.worldCoordToMap(req.goal_x, req.goal_y)

        start = self._graph[(start_x, start_y)]
        goal = self._graph[(goal_x, goal_y)]

        return self.compute(start, goal)

    def compute(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))

        frontier_nodes = [start]
        already_explored = [start]

        came_from = {start: None}

        cost_so_far = {start: 0}

        while not frontier.empty():
            queue_entry = frontier.get()
            current = queue_entry[1]

            if self.draw and (current in frontier_nodes):
                frontier_nodes.remove(current)

            for next_node in current.neighbors:
                # Get the current node from the graph
                next_node = self._graph[next_node]
                new_cost = cost_so_far[current] + self._costmap.data[current.y * self._costmap.info.height + current.x]

                if (next_node not in cost_so_far) or (new_cost < cost_so_far[next_node]):
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    frontier.put((priority, next_node))
                    came_from[next_node] = current

                    if self.draw and (next_node not in frontier_nodes):
                        frontier_nodes.append(next_node)

                if current not in already_explored:
                    already_explored.append(current)

            # Update visualization
            if self.draw:
                self.drawNodes(frontier_nodes, "frontier")
                self.drawNodes(already_explored, "explored")

            if current == goal:
                break

        # Work our way back through the came_from chain from the goal to the start
        path = [goal]
        previous_node = came_from[goal]
        while previous_node is not None:
            path.insert(0, previous_node)
            previous_node = came_from[previous_node]

        self.drawNodes(path, "path")

        # Compute ros Path
        waypoints = []
        index = 0
        current_dir = None
        for node in path:
            try:
                next_node = path[index + 1]

                # If we've changed directions, and add a waypoint if so
                this_dir = math.atan2((next_node.y - node.y), (next_node.x - node.x))
                if current_dir == None or this_dir != current_dir:
                    current_dir = this_dir

                    # copied some stuff from
                    # https://www.programcreek.com/python/example/70252/geometry_msgs.msg.PoseStamped
                    pose = PoseStamped()
                    pose.header.seq = len(waypoints)
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()

                    (xc, yc) = self.mapCoordsToWorld(node.x, node.y)

                    pose.pose.position.x = xc
                    pose.pose.position.y = yc
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

                (xc, yc) = self.mapCoordsToWorld(node.x, node.y)

                pose.pose.position.x = xc
                pose.pose.position.y = yc
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
        if topic == "frontier":
            publisher = self._grid_frontier_pub
        elif topic == "explored":
            publisher = self._grid_explored_pub
        elif topic == "path":
            publisher = self._grid_path_pub
        elif topic == "wall":
            publisher = self._grid_wall_pub
        else:
            return

        cells = GridCells()
        cells.header.frame_id = self._map.header.frame_id
        cells.cell_width = self._map.info.resolution
        cells.cell_height = self._map.info.resolution

        cell_points = []
        for node in nodes:
            (xc, yc) = self.mapCoordsToWorld(node.x, node.y)

            cell_x = xc
            cell_y = yc
            cell_points.append(Point(cell_x, cell_y, 0))

        cells.cells = cell_points
        publisher.publish(cells)

        rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node('astar_server')

    astar = AStar()
    s = rospy.Service('astar', group1.srv.AStar, astar.handle_request)

    rospy.spin()
