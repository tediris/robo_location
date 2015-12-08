import numpy
import math

def norm(vec):
	return numpy.linalg.norm(vec)


def getAngle(start, end):
	# get the vectors with respect to the start point
	path = numpy.array(end) - numpy.array(start)
	print path
	angle = math.degrees(math.atan2(-path[1], path[0])) - 90
	if angle < -180:
		return angle + 360
	return angle

'''
assume the points are all tuples
returns error, then dir
-1 means we are on track
0 means we are to the left
1 means we are to the right
2 means we are done moving
'''
def getError(start, end, curr):
	# get the vectors with respect to the start point
	location = numpy.array(curr) - numpy.array(start)
	path = numpy.array(end) - numpy.array(start)

	# normalize the travel direction
	pathMag = norm(path)
	pathDir = path
	if (pathMag != 0):
		pathDir = (1/pathMag) * path

	# if we've gone too far already, stop
	distMag = norm(location)
	if distMag > pathMag:
		return 10, 2

	# project the robot pos vector onto the direction
	projMag = numpy.dot(location, pathDir)
	proj = projMag * pathDir

	# see how far off we are from the direction
	errVec = location - proj
	err = norm(errVec)

	# if we have traveled far enough already
	closeThresh = 10.0
	if projMag > pathMag - closeThresh:
		return err, 2

	thresh = 5.0
	# if we are too far off:
	if (err > thresh):
		# figure out left or right
		dir = 0 # 0 is left, 1 is right

		# traveling left
		if path[0] < 0:
			# the robot is above the line
			if (errVec[1] > 0):
				dir = 1
		# traveling right
		elif path[0] > 0:
			if (errVec[1] < 0):
				dir = 1
		# along the vertical axis
		else:
			# traveling up
			if path[1] > 0:
				if errVec[0] > 0:
					dir = 1
			# traveling down
			else:
				if errVec[0] < 0:
					dir = 1

		# return the error and direction
		return err, dir
	# return the error, along with -1 to signal do nothing
	return err, -1
