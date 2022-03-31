class AntiCluster:
    def _init_(self, x, y):
        self._XLENGTH_ = x  # cm
        self._YLENGTH_ = y  # cm

    def aggregatePoints(self, points):

        for x in range(0, points.size()):
            xRangeTop = points[x].x + self._XLENGTH_
            yRangeTop = points[x].y + self._YLENGTH_
            xRangeBottom = points[x].x - self._XLENGTH_
            yRangeBottom = points[x].y - self._YLENGTH_

            for y in range(1, points.size()):
                if xRangeBottom < points[y].x < xRangeTop and yRangeBottom < points[y].y < yRangeTop:
                    points.pop(y)

        return points
