from geometry_msgs.msg import *
import rospy

class AntiCluster:
    def __init__(self, x, y, init_node=False):
        if init_node:
            rospy.init_node('aggregate_points')

        rospy.Subscriber("/trash_mapper/trash_points", Pose, self.addPoint)
        self._XLENGTH_ = x  # m
        self._YLENGTH_ = y  # m
        self.listOfPoints = []

    def addPoint(self, data):
        self.listOfPoints.append(data)
        self.listOfPoints = self.aggregatePoints(self.listOfPoints)
	print('Points: %s' % self.listOfPoints)

    def aggregatePoints(self, points):
        x = 0
        y = 1
        endX = len(points)

        while x < endX:
            y = x + 1
            xRangeTop = points[x].position.x + self._XLENGTH_
            yRangeTop = points[x].position.y + self._YLENGTH_
            xRangeBottom = points[x].position.x - self._XLENGTH_
            yRangeBottom = points[x].position.y - self._YLENGTH_

            while y < endX:
                if xRangeBottom <= points[y].position.x <= xRangeTop and yRangeBottom <= points[y].position.y <= yRangeTop:
                    #print(points[x])
                    #print(points[y])
                    points.pop(y)
                    endX = endX - 1
                y = y + 1

            x = x + 1

        return points


if __name__ == "__main__":
    '''
    first = [(1, 2), (20, 4), (3, 20), (20, 20), (1, 1), (21, 20), (40, 40)]
    print(first)
    
    cluster = AntiCluster(0.2, 0.2, True)
    last = cluster.aggregatePoints(first)
    print(last)
    '''
    cluster = AntiCluster(0.2,0.2, True)

    while True:
        continue
