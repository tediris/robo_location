'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import signal
import threading
import time
import numpy
import ast
import cv2
# import Tkinter as tk
import Queue
from collections import deque

import control
import pen
import drive

from ar_markers.hamming.detect import detect_marker
from HamsterAPI.comm_ble import RobotComm
#from HamsterAPI.comm_usb import RobotComm
# import draw
# from Behavior import motion, color, sound, proxy, scanning

gFrame = None
gQuit = Queue.Queue()
gBehaviors = {}
drawList = []

info = deque(maxlen=1)

def quit():
    print "quitting..."
    for i in range(0, len(gBehaviors)):
        gBehaviors[i].set_bQuit(True)
    time.sleep(1)
    gQuit.put("STOP")
    # gFrame.quit()

def clean_up():
    quit()
    print "cleaning up..."

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    clean_up()

def get_to_point(point, wheels):
    #drawList.append(point)
    rotate_towards_point(point, wheels)
    move_to_point(point, wheels)

def rotate_towards_point(point, wheels):
    print "ROTATING"
    while len(info) == 0:
        time.sleep(0.1)
    (location, rotation) = info.pop()#infoQ.get()
    #drawList.append((int(location[0]), int(location[1])))
    angle = control.getAngle(location, point)
    wheels.drive(-5, 5)
    while (abs(rotation - angle) > 2) and gQuit.empty():
        #print "Angle: " + str(angle)
        #print "Rotation: " + str(rotation)
        while len(info) == 0:
            time.sleep(0.1)
        (location, rotation) = info.pop()#infoQ.get()
    wheels.stop()

def move_to_point(point, wheels):
    print "MOVE FORWARD"
    while len(info) == 0:
        time.sleep(0.1)
    (location, rotation) = info.pop()#infoQ.get()
    start = location
    error, dir = control.getError(start, point, location)
    #drawList.append(location)
    while (not dir == 2) and gQuit.empty():
        if dir == -1:
            wheels.drive(10, 10)
        elif dir == 1:
            wheels.drive(11, 9)
        elif dir == 0:
            wheels.drive(9, 11)
        try:
            while len(info) == 0:
                time.sleep(0.1)
            (location, rotation) = info.pop()#infoQ.get(False)
        except Queue.Empty:
            pass
        error, dir = control.getError(start, point, location)
        #drawList[-1] = location
        time.sleep(0.1)
    wheels.stop()

def main_thread():
    size = (960, 540)
    capture = cv2.VideoCapture(0)

    if capture.isOpened(): # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False
    while frame_captured and gQuit.empty():
        marker = detect_marker(frame)
        if marker != None:
            marker.highlite_marker(frame)
            location, rotation = marker.get_location_rotation()
            info.append((location, rotation))
        for item in drawList:
            cv2.circle(frame, item, 5, (0,255,255), 4)
        frameSmall = cv2.resize(frame, size)
        cv2.imshow('Test Frame', frameSmall)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()
        time.sleep(0.01)
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

def command(paths, robotList):
    wheels = drive.Drive(robotList)
    wheels.waitForConn(gQuit)

    marker = pen.Pen(robotList)
    marker.waitForConn(gQuit)
    time.sleep(2)
    marker.lower()

    marker.lift()
    # unaligned drawing, faster
    for path in paths:
        startLoc = path[0]
        get_to_point(startLoc, wheels)
        for i in range(1, len(path)):
            destLoc = path[i]
            print 'Destination: ' + str(destLoc)
            rotate_towards_point(destLoc, wheels)
            marker.lower()
            move_to_point(destLoc, wheels)
            marker.lift()
            # move to the new point

    # aligned drawing, slower but more accurate
    # for path in paths:
    #     startLoc = path[0]
    #     for i in range(0, len(path) - 1):
    #         vecToPoint = numpy.array(path[i + 1]) - numpy.array(path[i])
    #         unitVec = (1 / numpy.linalg.norm(vecToPoint)) * vecToPoint
    #         offset = unitVec * 20
    #         startLoc = tuple(numpy.add(path[i], offset))
    #         destLoc = tuple(numpy.add(path[i + 1], offset))
    #         get_to_point(startLoc, wheels)
    #         #if (i == 0):
    #         rotate_towards_point(destLoc, wheels)
    #         #else:
    #         #    rotate_towards_point(destLoc, wheels, "right")
    #         marker.lower()
    #         move_to_point(destLoc, wheels)
    #         marker.lift()
    #         # move to the new point

    print "Closing command"

def getPaths(camWidth, camHeight, camXOffset, camYOffset):
    name = "momosavestheday.txt"
    f = open(name, 'r')
    content = f.readlines()
    dimensions = content[0].split(",")
    imgWidth = int(dimensions[0])
    imgHeight = int(dimensions[1])
    ratio = min((float(camWidth) / imgWidth), (float(camHeight) / imgHeight))
    paths = []
    for i in range(1, len(content)):
        strsplit = content[i].split(" ")
        path = []
        for item in strsplit:
            if len(item) < 3:
                continue
            point = ast.literal_eval(item)
            newX = int(point[0] * ratio + camXOffset)
            newY = int(point[1] * ratio + camYOffset)
            path.append((newX, newY))
            drawList.append((newX, newY))

        paths.append(path)
    f.close()
    return paths

signal.signal(signal.SIGINT, signal_handler)

def main(argv=None):

    paths = getPaths(1000, 600, 500, 200)
    drawList.append((500, 200))
    drawList.append((1500, 200))
    drawList.append((500, 800))
    drawList.append((1500, 800))

    # instantiate COMM object
    comm = RobotComm(2, -50) #maxRobot = 1, minRSSI = -50
    if comm.start():
        print 'Communication starts'
    else:
        print 'Error: communication'
        return

    # instantiate Robot
    robotList = comm.get_robotList()

    # create behaviors using set
    global gBehaviors
    gBehaviors = {}

    # start behavior threads using list
    behavior_threads = []
    behavior_threads.append(threading.Thread(target=command, args=(paths, robotList)))

    for thread in behavior_threads:
        thread.daemon = True
        thread.start()

    # start the cv thread
    main_thread()

    for behavior in behavior_threads:
        print "joining... ", behavior.getName()
        behavior.join()
        print behavior.getName(), "joined!"

    for robot in robotList:
        robot.reset()


    comm.stop()
    comm.join()

    print("terminated!")

if __name__ == "__main__":
    sys.exit(main())
