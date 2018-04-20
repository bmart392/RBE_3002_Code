#!/usr/bin/env python
import rospy, math, copy
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
from astar import AStar, Node


class Explore:

    def __init__(self):

        self._map = OccupancyGrid()
        self._mapComplete = False
        self._currentPosX = 0.0
        self._currentPosY = 0.0

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.updatemap, queue_size=1)
        rospy.Subscriber('/robot_pos', Pose, self.updatepos, queue_size=1)

        # Publishers
        self._frontier_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True, queue_size=1)
        self._grid_frontier_pub = rospy.Publisher('/astar_grid_frontier', GridCells, latch=True, queue_size=10)
        self._grid_path_pub = rospy.Publisher('/astar_grid_path', GridCells, latch=True, queue_size=1)

    def updatemap(self, new_map):
        self._map = copy.deepcopy(new_map)
        self._mapComplete = False
        print "Map updated"

    def updatepos(self, msg):
        if not self._map.info.resolution == 0:
            current_pos_world = copy.deepcopy(msg)
            current_pos_map = self.worldCoordToMap(current_pos_world.position.x, current_pos_world.position.y)
            self._currentPosX = current_pos_map[0]
            self._currentPosY = current_pos_map[1]

    def explore(self, event):

        frontier_nodes = []

        if not self._mapComplete:
            for x in range(0, self._map.info.width):
                for y in range(0, self._map.info.height):
                    cell_value = self._map.data[y * self._map.info.height + x]
                    if 95 > cell_value > -1: # and not self.checkNeighborsForWall(x, y):
                        neighbors = self.getNeighborsWithoutWalls(x, y)
                        if len(neighbors) < 8:
                            break
                        for coord in neighbors:
                            neighbor_value = self._map.data[coord[1] * self._map.info.height + coord[0]]
                            if neighbor_value < 0:
                                frontier_nodes.append(Node(x, y, []))
                                break


        if not frontier_nodes:
            #self._mapComplete = True
            print("Frontier is empty")
            return

        frontier_nodes_filtered = frontier_nodes
        # frontier_nodes_filtered = []
        # num_neighbors_in_frontier = 0
        # for node_for_neighbors in frontier_nodes:
        #     node_neighbors = self.getNeighborsWithWalls(node_for_neighbors.x, node_for_neighbors.y)
        #     for node_in_frontier in frontier_nodes:
        #         for coord in node_neighbors:
        #                 if node_in_frontier.x == coord[0] and node_in_frontier.y == coord[1]:
        #                     num_neighbors_in_frontier += 1
        #                     if num_neighbors_in_frontier > 2:
        #                         frontier_nodes_filtered.append(node_for_neighbors)
        #                         num_neighbors_in_frontier = 0

        self.drawNodes(frontier_nodes_filtered, "frontier")

        closest_node = frontier_nodes_filtered[0]
        closest_distance = math.sqrt((self._currentPosX - closest_node.x)**2 + (self._currentPosY - closest_node.y)**2)
        for node in frontier_nodes_filtered:
            node_distance = math.sqrt((self._currentPosX - node.x) ** 2 + (self._currentPosY - node.y) ** 2)
            if node_distance < closest_distance:
                closest_distance = math.sqrt((self._currentPosX - node.x)**2 + (self._currentPosY - node.y)**2)
                closest_node = node

        self.drawNodes([closest_node] * 2, "path")

        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()

        p = self.mapCoordsToWorld(closest_node.x, closest_node.y)
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
        neighbors = []
        # for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                continue

            if self._map.data[yc * self._map.info.height + xc] < 100:
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
    rospy.Timer(rospy.Duration(3), turtle.explore)
   # turtle.explore
    rospy.spin()



