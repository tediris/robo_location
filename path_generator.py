#This file will contain code for taking in an image and returning a queue of paths for the robot to follow
# PLEASE TAKE CARE - EVERYTHING IS IN COORDINATES (ROW,COLUMN) --> IN OTHER WORDS, (Y,X)
from PIL import Image
import Queue
import numpy
import copy

DARK_VAL = 150
UP = (-1,0)
RIGHT = (0,1)
DOWN = (1,0)
LEFT = (0,-1)
TURN_RIGHT = ((True, True),(True, False))
GO_STRAIGHT = ((True, False),(True, False))


def getPixelArrayForFilename(filename):
    image = Image.open(filename, 'r')
    if (not image):
        print "bad filename"
        return
    pixelList = list(image.getdata())
    width = image.size[0]
    height = image.size[1]
    array = [[0 for x in range(0,width)] for x in range(0,height)]
    for r in range(0, height):
        for c in range(0, width):
            array[r][c] = pixelList[r*width + c]
    return array

def printPixelArray(array):
    for r in range(0, len(array)):
        row = ""
        for c in range(0, len(array[r])):
            row = row + str(array[r][c]) + " "
        print row

def print01Array(array):
    for r in range(0, len(array)):
        row = ""
        for c in range(0, len(array[r])):
            if array[r][c]:
                row = row + "1 "
            else:
                row = row + "0 "
        print row

def printBlackEnoughArray(array):
    for r in range(0, len(array)):
        row = ""
        for c in range(0, len(array[r])):
            if isBlackEnough(array[r][c]):
                row = row+" 1"
            else:
                row = row+" 0"
        print row

def isBlackEnough(pixel):
    return pixel[0] < DARK_VAL and pixel[1] < DARK_VAL and pixel[2] < DARK_VAL

def getBoolArray(array):
    height = len(array)
    width = len(array[0])
    boolArray = [[0 for x in range(0,width)] for x in range(0,height)]
    for r in range(0, height):
        for c in range(0, width):
            boolArray[r][c] = isBlackEnough(array[r][c])
    return boolArray

def getRawPaths(array):
    allPaths = []
    height = len(array)
    width = len(array[0])
    for r in range(0, height):
        for c in range(0, width):
            # if len(allPaths) > 25:
            #     return allPaths
            if array[r][c]:
                print "Getting path " + str(len(allPaths) + 1)
                vec = []
                vec.append(((r,c),(r+1,c)))
                getNextStep((r+1, c), (r, c), array, vec)
                allPaths.append(vec)
                mask = getInversionMask(vec, array)
                # printPixelArray(mask)
                invertForMask(array, mask)
                # print01Array(array)
                r = 0
                c = 0
    return allPaths

def getNextStep(current, start, array, vec):
    # print vec
    if current == start:
        return
    lastEdge = vec[len(vec)-1]
    direction = tuple(numpy.subtract(lastEdge[1],lastEdge[0]))
    square = getSurroundingSquare(lastEdge[1], array, direction)
    # print printPixelArray(square)
    if (square == TURN_RIGHT):
        # turn right
        # print "Turning Right"
        if direction == UP:
            nextPoint = tuple(numpy.add(current, (0,1)))
        if direction == RIGHT:
            nextPoint = tuple(numpy.add(current, (1,0)))
        if direction == DOWN:
            nextPoint = tuple(numpy.add(current, (0,-1)))
        if direction == LEFT:
            nextPoint = tuple(numpy.add(current, (-1,0)))
    elif (square == GO_STRAIGHT):
        # print "Going Straight"
        nextPoint = tuple(numpy.add(current, direction))
    else:
        # go left
        # print "Going Left"
        if direction == UP:
            nextPoint = tuple(numpy.add(current, (0,-1)))
        if direction == RIGHT:
            nextPoint = tuple(numpy.add(current, (-1,0)))
        if direction == DOWN:
            nextPoint = tuple(numpy.add(current, (0,1)))
        if direction == LEFT:
            nextPoint = tuple(numpy.add(current, (1,0)))
    vec.append((current, nextPoint))
    getNextStep(nextPoint,start,array,vec)

# rotates 90 degrees to the right
def rotated(array):
    return tuple(zip(*array[::-1]))

#orients it to always act as if our direction is 'up' for comparison to TURN_RIGHT/GO_STRAIGHT
def getSurroundingSquare((r,c), array, direction):
    square = ((array[r-1][c-1], array[r-1][c]),(array[r][c-1], array[r][c]))
    if direction == UP:
        return square
    square = rotated(square)
    if direction == LEFT:
        return square
    square = rotated(square)
    if direction == DOWN:
        return square
    return rotated(square)

def getDirection(edge):
    return tuple(numpy.subtract(edge[1],edge[0]))

def getInversionMask(path, array):
    height = len(array)
    width = len(array[0])
    mask = [[0 for x in range(0,width)] for x in range(0,height)]
    for edge in path:
        direction = getDirection(edge)
        if direction == DOWN:
            mask[edge[0][0]][edge[0][1]] = 1
        elif direction == LEFT:
            mask[edge[1][0]][edge[1][1]] = 1
        elif direction == UP:
            mask[edge[1][0]][edge[1][1] - 1] = 1
        else:
            mask[edge[0][0] - 1][edge[0][1]] = 1
    firstEdge = path[0]
    secondEdge = path[1]
    secondDir = getDirection(secondEdge)

    outline = copy.deepcopy(mask)

    floodFill(mask, tuple(numpy.add(firstEdge[1], (-1,-1))))
    # printPixelArray(mask)

    for r in range(0, height):
        for c in range(0, width):
            if outline[r][c] == 1:
                mask[r][c] = 1
    # printPixelArray(mask)

    return mask

def floodFill(mask, loc):
    height = len(mask)
    width = len(mask[0])
    q = Queue.Queue()
    q.put(loc)
    while not q.empty():
        loc = q.get()
        if loc[0] < 0 or loc[1] < 0 or loc[0] >= height or loc[1] >= width or mask[loc[0]][loc[1]] == 1:
            continue
        mask[loc[0]][loc[1]] = 1
        q.put(tuple(numpy.add(loc, UP)))
        q.put(tuple(numpy.add(loc, RIGHT)))
        q.put(tuple(numpy.add(loc, LEFT)))
        q.put(tuple(numpy.add(loc, DOWN)))
    if mask[0][0] == 1:
        for r in range(0, height):
            for c in range(0, width):
                if mask[r][c] == 1:
                    mask[r][c] = 0
                else:
                    mask[r][c] = 1
    # if mask[loc[0]][loc[1]] == 1:
    #     return
    # mask[loc[0]][loc[1]] = 1
    # floodFill(mask, tuple(numpy.add(loc, UP)))
    # floodFill(mask, tuple(numpy.add(loc, RIGHT)))
    # floodFill(mask, tuple(numpy.add(loc, LEFT)))
    # floodFill(mask, tuple(numpy.add(loc, DOWN)))

def invertForMask(array, mask):
    height = len(array)
    width = len(array[0])
    for r in range(0, height):
        for c in range(0, width):
            if mask[r][c] == 1:
                array[r][c] = not array[r][c]

def getD(coord1, coord2):
    return max(abs(coord1[0] - coord2[0]), abs(coord1[1] - coord2[1]))

def lineMagnitude (x1, y1, x2, y2):
    lineMagnitude = numpy.sqrt(numpy.power((x2 - x1), 2)+ numpy.power((y2 - y1), 2))
    return lineMagnitude

#Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
def DistancePointLine (point, start, end):
    #http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
    px = point[1]
    py = point[0]
    x1 = start[1]
    y1 = start[0]
    x2 = end[1]
    y2 = end[0]
    LineMag = lineMagnitude(x1, y1, x2, y2)

    if LineMag < 0.00000001:
        DistancePointLine = 9999
        return DistancePointLine

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (LineMag * LineMag)

    if (u < 0.00001) or (u > 1):
        #// closest point does not fall within the line segment, take the shorter distance
        #// to an endpoint
        ix = lineMagnitude(px, py, x1, y1)
        iy = lineMagnitude(px, py, x2, y2)
        if ix > iy:
            DistancePointLine = iy
        else:
            DistancePointLine = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        DistancePointLine = lineMagnitude(px, py, ix, iy)

    return DistancePointLine

def DistTo(coord1, coord2):
    return numpy.power(coord1[0]-coord2[0], 2) + numpy.power(coord1[1]-coord2[1], 2)

def getDistance(coord1, coord2):
    return numpy.sqrt(DistTo(coord1, coord2))

def getDistToLine(start, end, point, L):
    T = ((point[0]-start[0])*(end[0]-start[0]) + (point[1]-start[1])*(end[1]-start[1]))/L
    print T
    return DistTo(point, (start[0]+T*(end[0]-start[0]), start[1]+T*(end[1]-start[1])))

def isStraight(path, smoothFactor):
    if len(path) == 0 or len(path) == 1:
        return True
    start = path[0][0]
    end = path[len(path) - 1][0]
    L = DistTo(start, end)
    for edge in path:
        point = edge[0]
        if DistancePointLine(point, start, end) > smoothFactor:
            return False
    return True

def combinePaths(allPaths, smoothFactor = 1): #DON'T KNOW IF YOU CAN CHANGE smoothFactor WITHOUT MESSING EVERYTHING UP
    newPaths = []
    i = 1
    for path in allPaths:
        print "Compressing path " + str(i)
        newPaths.append(combinePath(path, smoothFactor))
        i = i + 1
    return newPaths

def combinePath(path, smoothFactor):
    permCopy = copy.deepcopy(path)
    newPath = []
    # length = len(path)
    start = 0
    end = 3
    while end < len(path):
        while end < len(path) and isStraight(path[start:end], smoothFactor):
            end = end + 1
        end = end - 1
        newPath.append((path[start][0], path[end][0]))
        start = end
        end = start + 2
    newPath.append((path[start][0], path[0][0]))
    return newPath

def getCompressedPaths(filename, smoothFactor):
    array = getPixelArrayForFilename(filename)
    width = len(array[0])
    height = len(array)
    array = getBoolArray(array)
    paths = getRawPaths(array)
    paths = combinePaths(paths, smoothFactor)
    return paths, width, height

def saveToFile(filename, paths, width, height):
    f = open(filename, 'w')
    f.write(str(width) + ',' + str(height) + "\n")
    #widht and height
    for path in paths:
        pathString = ""
        for edge in path:
            pathString = pathString + "(" + str(edge[0][0]) + "," + str(edge[0][1]) + ") "
        pathString = pathString + "(" + str(path[0][0][0]) + "," + str(path[0][0][1]) + ")\n"
        f.write(pathString)
