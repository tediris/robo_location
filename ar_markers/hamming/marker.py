import cv2

from numpy import mean, binary_repr, zeros
from numpy.random import randint
from scipy.ndimage import zoom

from ar_markers.hamming.coding import encode, HAMMINGCODE_MARKER_POSITIONS

MARKER_SIZE = 7


class HammingMarker(object):
    def __init__(self, id, contours=None, rotation=0):
        self.id = id
        self.contours = contours
        self.rotation = rotation # in degrees

    def __repr__(self):
        return '<Marker id={} center={}>'.format(self.id, self.center)

    @property
    def center(self):
        if self.contours is None:
            return None
        center_array = mean(self.contours, axis=0).flatten()
        return (int(center_array[0]), int(center_array[1]))

    def generate_image(self):
        img = zeros((MARKER_SIZE, MARKER_SIZE))
        img[1, 1] = 255  # set the orientation marker
        for index, val in enumerate(self.hamming_code):
            coords = HAMMINGCODE_MARKER_POSITIONS[index]
            if val == '1':
                val = 255
            img[coords[0], coords[1]] = int(val)
        return zoom(img, zoom=50, order=0)

    def draw_contour(self, img, color=(0, 255, 0), linewidth=5):
        cv2.drawContours(img, [self.contours], -1, color, linewidth)

    def highlite_marker(self, img, contour_color=(0, 255, 0), text_color=(255, 0, 0), linewidth=5):
        self.draw_contour(img, color=contour_color, linewidth=linewidth)
        cv2.putText(img, str(self.id), self.center, cv2.FONT_HERSHEY_SIMPLEX, 2, text_color)

    def get_location_rotation(self):
        xMean = 0
        yMean = 0
        numPoints = len(self.contours)
        for point in self.contours:
            xMean += point[0][0]
            yMean += point[0][1]

        xMean = xMean / numPoints
        yMean = yMean / numPoints
        rotation = self.fix_rotation()
        return (xMean, yMean), rotation

    def fix_rotation(self):
        angle = self.rotation
        if self.rotation > -90 and self.rotation < 0:
            angle = 180 + self.rotation
        if self.rotation > 90 and self.rotation < 180:
            angle = self.rotation - 180
        angle = angle - 180
        if angle < -180:
    		return angle + 360
        return angle

    def print_center(self):
        xMean = 0
        yMean = 0
        numPoints = len(self.contours)
        for point in self.contours:
            xMean += point[0][0]
            yMean += point[0][1]

        xMean = xMean / numPoints
        yMean = yMean / numPoints
        #print '---------------'
        print '[' + str(xMean) + ', ' + str(yMean) + '], ' + str(self.rotation)

    @classmethod
    def generate(cls):
        return HammingMarker(id=randint(4096))

    @property
    def id_as_binary(self):
        return binary_repr(self.id, width=12)

    @property
    def hamming_code(self):
        return encode(self.id_as_binary)
