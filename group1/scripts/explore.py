#!/usr/bin/env python
import rospy, math, copy, tf
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import quaternion_from_euler
from astar import AStar, Node


class Explore:

    def __init__(self):

        self._map = OccupancyGrid()
        self._mapComplete = False

        self._closest_node = None
        self._current_node = Node(0, 0, [])

        self._goal_reached = True
        self._node_blacklist = []

        self._currentPosX = 0.0
        self._currentPosY = 0.0

        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.updateCurrentPose)
        rospy.Timer(rospy.Duration(1), self.update_frontier)
        rospy.Timer(rospy.Duration(.5), self.choose_new_goal)

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.updatemap, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.checkStatus, queue_size=1)

        # Publishers
        self._frontier_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True, queue_size=1)
        self._grid_frontier_pub = rospy.Publisher('/astar_grid_frontier', GridCells, latch=True, queue_size=10)
        self._grid_path_pub = rospy.Publisher('/astar_grid_path', GridCells, latch=True, queue_size=1)


    def updateCurrentPose(self, evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """

        # wait for and get the transform between two frames
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('odom', 'base_link', rospy.Time(0))

        # check if map is there
        if self._map.info.resolution == 0:
            return

        # save the current position
        current_pos_map = self.worldCoordToMap(position[0], position[1])

        self._currentPosX = current_pos_map[0]
        self._currentPosY = current_pos_map[1]

    def checkStatus(self, _status):
        for status in _status.status_list:
            if status.status == 3:
                self._goal_reached = True
                print "Reset GR 3"
                break
            elif status.status == 4: # If we can't find a path to a node, then we add it to the blacklist
                if self._current_node not in self._node_blacklist:
                    self._node_blacklist.append(self._current_node)
                self._goal_reached = True
                print "Reset GR 4"

    def updatemap(self, new_map):
        self._map = copy.deepcopy(new_map)
        self._mapComplete = False

    def update_frontier(self, event):
        # Don't explore if map is completed
        if self._mapComplete:
            return

        frontier_nodes = []
        closest_distance = float("inf")
        for x in range(0, self._map.info.width):
            for y in range(0, self._map.info.height):
                cell_value = self._map.data[y * self._map.info.height + x]
                if 95 > cell_value > -1: # and not self.checkNeighborsForWall(x, y):
                    neighbors = self.getNeighborsWithoutWalls(x, y)

                    if len(neighbors) < 8:
                        continue

                    for coord in neighbors:
                        neighbor_value = self._map.data[coord[1] * self._map.info.height + coord[0]]
                        node = Node(x, y, [])
                        if (neighbor_value < 0) and (node not in self._node_blacklist):
                            frontier_nodes.append(node)

                            node_distance = math.sqrt((self._currentPosX - node.x)**2 + (self._currentPosY - node.y)**2)
                            if node_distance < closest_distance:
                                closest_distance = node_distance
                                self._closest_node = node

                            break

        if not frontier_nodes:
            #self._mapComplete = True
            print("Frontier is empty")
            return

        self.drawNodes(frontier_nodes, "frontier")

    def choose_new_goal(self, event):
        # Don't choose a new goal if we haven't reached the last goal
        if not self._goal_reached or self._closest_node is None:
            return

        self._goal_reached = False

        print "Choosing new goal"

        self.drawNodes([self._closest_node] * 2, "path")
        self._current_node = self._closest_node

        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()

        p = self.mapCoordsToWorld(self._closest_node.x, self._closest_node.y)
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation = Quaternion(*q)

        self._frontier_pub.publish(pose)

    def getNeighborsWithWalls(self, x, y):
        neighbors = []
        # for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                continue

            neighbors.append(coord)

        return neighbors

    def getNeighborsWithoutWalls(self, x, y):
        return self._getNeighborsWithoutWalls(x, y, 2)

    def _getNeighborsWithoutWalls(self, x, y, recurse):
        neighbors = []
        # for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                continue

            if self._map.data[yc * self._map.info.height + xc] < 100:
                add_neighbor = True
                if recurse != 0:
                    recurse -= 1
                    add_neighbor = (len(self._getNeighborsWithoutWalls(xc, yc, recurse)) == 8)

                if add_neighbor:
                    neighbors.append(coord)

        return neighbors

    def mapCoordsToWorld(self, x, y):
        xc = (x * self._map.info.resolution) + (self._map.info.resolution / 2) + (self._map.info.origin.position.x)
        yc = (y * self._map.info.resolution) + (self._map.info.resolution / 2) + (self._map.info.origin.position.y)

        return xc, yc

    def worldCoordToMap(self, x, y):
        xc = int((x - self._map.info.origin.position.x) / self._map.info.resolution)
        yc = int((y - self._map.info.origin.position.y) / self._map.info.resolution)

        return xc, yc

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

    def checkNeighborsForWall(self, x, y):
        neighbors = self.getNeighborsWithWalls(x, y)
        for coord in neighbors:
            cell_val = self._map.data[coord[0] * self._map.info.height + coord[1]]
            if cell_val > 50:
                return True


if __name__ == '__main__':
    rospy.init_node('explore_node')
    turtle = Explore()
    rospy.spin()



