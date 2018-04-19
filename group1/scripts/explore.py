#!/usr/bin/env python
import rospy, math, copy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from astar import AStar, Node
from robot import Robot


class Explore:

    def __init__(self):

        self._map = OccupancyGrid()
        self._mapComplete = False

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.updatemap, queue_size=1)

        # Publishers
        self._frontier_pub = rospy.Publisher('/move_base/simple_goal', PoseStamped, latch=True, queue_size=1)

    def updatemap(self,new_map):
        self._map = copy.deepcopy(new_map)
        self._mapComplete = False
        self.explore()

    def explore(self):

        frontier_nodes = []

        if not self._mapComplete:
            for x in range(0, self._map.info.width):
                for y in range(0, self._map.info.height):
                    if 95 > self._map.data[y * self._map.info.height + x] > -1:
                        neighbors = self.getNeighbors(x, y)
                        for coord in neighbors:
                            if self._map.data[coord[1] * self._map.info.height + coord[0]] < 0:
                                frontier_nodes.append(Node(x, y, []))

        if not frontier_nodes:
            self._mapComplete = True
            print("Frontier is empty")
            return

        #distance = 100  # Random number that will guarantee the distance is always smaller. Probably need to TODO make better

        closest_node = frontier_nodes[0]
        closest_distance = math.sqrt((x ** 2 - closest_node.x ** 2) + (y ** 2 - closest_node.y ** 2))
        for node in frontier_nodes:
            if math.sqrt((x ** 2 - node.x ** 2) + (y ** 2 - node.y ** 2)) < closest_distance:
                closest_distance = math.sqrt((x ** 2 - node.x ** 2) + (y ** 2 - node.y ** 2))
                closest_node = node

                # node is where we want to go
        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = closest_node.x
        pose.pose.position.y = closest_node.y
        pose.pose.position.z = 0

        pose.pose.orientation.w = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        frontier_goal = pose

        self._frontier_pub.publish(frontier_goal)

    # def mapComplete(self):
    #
    #     for x in range(0, self._map.info.width):
    #         for y in range(0, self._map.info.height):
    #             if 50 < self.map.data[y * self._map.info.height + x] < 95:
    #                 return True

    # def checkNeighbors(self,x,y):
    #     neighbors = self.getNeighbors(x,y)
    #     node =
    #     for  in neighbors

    def getNeighbors(self, x, y):
        neighbors = []
        # for coord in [(x, y - 1),  (x - 1, y), (x + 1, y), (x, y + 1)]:
        for coord in [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y),
                      (x + 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]:
            xc = coord[0]
            yc = coord[1]

            if xc < 0 or yc < 0 or xc >= self._map.info.width or yc >= self._map.info.height:
                continue

            if self._map.data[yc * self._map.info.height + xc] < 95:
                neighbors.append(coord)

        return neighbors


if __name__ == '__main__':
    rospy.init_node('explore_node')
    turtle = Explore()
    rospy.spin()



