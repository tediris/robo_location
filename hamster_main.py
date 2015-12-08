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
# import Tkinter as tk
import Queue
import control
import numpy
import ast

import cv2

from ar_markers.hamming.detect import detect_markers

from HamsterAPI.comm_ble import RobotComm
#from HamsterAPI.comm_usb import RobotComm
# import draw
# from Behavior import motion, color, sound, proxy, scanning

gFrame = None
gQuit = Queue.Queue()
gBehaviors = {}
armQ = Queue.Queue()
timeQ = Queue.Queue()
driveQ = Queue.Queue()
infoQ = Queue.Queue()
travelQ = Queue.Queue()
doneQ = Queue.Queue()
drawList = []

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

def drive(robotList):
    print "DRIVE"

    drive = [0,0]

    while gQuit.empty():
        if len(robotList) > 0:
            robotList[0].set_wheel(0,drive[0])
            robotList[0].set_wheel(1,drive[1])
        if not driveQ.empty():
            drive = driveQ.get()
        time.sleep(0.1)
    print "Closing drive"

def arm(robotList):
    while gQuit.empty():
        if len(robotList) > 1:
            # lift_pen(robotList)
            # lower_pen(robotList)
            if not armQ.empty():
                command = armQ.get()
                if command == "LIFT":
                    lift_pen(robotList)
                elif command == "LOWER":
                    lower_pen(robotList)
            robotList[1].set_wheel(0, 0)
            robotList[1].set_wheel(1, 0)
        time.sleep(0.1)
    print "Closing arm"

def timer():
    while gQuit.empty():
        if not timeQ.empty():
            timeQ.get()
            time.sleep(1.0)
            armQ.put("DONE")
        time.sleep(0.1)
    print "Closing timer"

def traveler():
    while gQuit.empty():
        if not travelQ.empty():
            travelTime = travelQ.get()
            time.sleep(travelTime)
            doneQ.put("DONE")
        time.sleep(0.1)
    print "Closing traveler"

def lift_pen(robotList):
    timeQ.put("START")
    while armQ.empty() and gQuit.empty():
        robotList[1].set_wheel(0, 30)
        robotList[1].set_wheel(1, 30)
    armQ.get()

def lower_pen(robotList):
    timeQ.put("START")
    while armQ.empty() and gQuit.empty():
        robotList[1].set_wheel(0, -30)
        robotList[1].set_wheel(1, -30)
    armQ.get()

def get_to_point(point):
    #drawList.append(point)
    rotate_towards_point(point)
    move_to_point(point)

def rotate_towards_point(point):
    (location, rotation) = infoQ.get()
    #drawList.append((int(location[0]), int(location[1])))
    angle = control.getAngle(location, point)
    print 'Angle to Point: ' + str(angle)
    while (abs(rotation - angle) > 2) and gQuit.empty():
        #print "Angle: " + str(angle)
        #print "Rotation: " + str(rotation)
        if driveQ.empty():
            driveQ.put([-5, 5])
        (location, rotation) = infoQ.get()
    driveQ.put([0,0])

def move_to_point(point):
    while not doneQ.empty():
        doneQ.get()
    print "MOVE FORWARD"
    (location, rotation) = infoQ.get()
    start = location
    error, dir = control.getError(start, point, location)
    #travelQ.
    while (not dir == 2) and gQuit.empty() and doneQ.empty():
        if dir == -1:
            driveQ.put([10,10])
        elif dir == 1:
            driveQ.put([11,9])
        elif dir == 0:
            driveQ.put([9,11])
        try:
            (location, rotation) = infoQ.get(False)
        except Queue.Empty:
            pass
        error, dir = control.getError(start, point, location)
        time.sleep(0.1)
    driveQ.put([0,0])

def main_thread():
    capture = cv2.VideoCapture(0)

    if capture.isOpened(): # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False
    while frame_captured and gQuit.empty():
        markers = detect_markers(frame)
        for marker in markers:
            marker.highlite_marker(frame)
            #marker.print_center()
            location, rotation = marker.get_location_rotation()
            # if not infoQ.empty():
            #     infoQ.get()
            infoQ.put((location, rotation))
        for item in drawList:
            cv2.circle(frame, item, 5, (0,255,255), 4)
        # print frame.shape, frame.dtype
        size = (960, 540)
        # m = numpy.zeros(size, dtype=numpy.uint8)
        frameSmall = cv2.resize(frame, size)
        cv2.imshow('Test Frame', frameSmall)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()
        # time.sleep(0.1)
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

def command(paths):

    for path in paths:
        startLoc = path[0]
        # armQ.put("LIFT")
        get_to_point(startLoc)
        for i in range(1, len(path)):
            destLoc = path[i]
            print 'Destination: ' + str(destLoc)
            rotate_towards_point(destLoc)
            # armQ.put("LOWER")
            move_to_point(destLoc)
            # armQ.put("LIFT")


    # pointA = (900, 300)
    # armQ.put("LOWER")
    # get_to_point(pointA)

    # time.sleep(15)
    # print "START COMMAND"
    # driveQ.put([5,5])
    # while gQuit.empty():
    #     print 'lifting'
    #     armQ.put("LIFT")
    #     time.sleep(3)
    #     print 'lowering'
    #     armQ.put("LOWER")
    #     time.sleep(3)
    # driveQ.put([0,0])

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
    return paths

signal.signal(signal.SIGINT, signal_handler)

def main(argv=None):

    paths = getPaths(1000, 600, 500, 200)
    drawList.append((500, 200))
    drawList.append((1500, 200))
    drawList.append((500, 800))
    drawList.append((1500, 800))

    # instantiate COMM object
    comm = RobotComm(1, -50) #maxRobot = 1, minRSSI = -50
    if comm.start():
        print 'Communication starts'
    else:
        print 'Error: communication'
        return

    # instanciate Robot
    robotList = comm.get_robotList()

    # global gFrame
    # gFrame = tk.Tk()
    # gFrame.geometry('600x500')
    #gFrame.focus_set()
    #gFrame.bind('<KeyPress>', joystick)

    # gRobotDraw = draw.RobotDraw(gFrame, tk)

    # create behaviors using set
    global gBehaviors
    gBehaviors = {}
    # gBehaviors[0] = color.Behavior("color", robotList)
    # gBehaviors[1] = sound.Behavior("sound", robotList)
    # gBehaviors[2] = motion.Behavior("motion", robotList, 10)
    # gBehaviors[3] = proxy.Behavior("proxy", robotList, 85, 0.01, gRobotDraw.get_queue())
    # gBehaviors[0] = scanning.Behavior("scanning", robotList, 16.0, gRobotDraw.get_queue())

    # start behavior threads using list
    behavior_threads = []
    behavior_threads.append(threading.Thread(target=drive, args=(robotList, )))
    behavior_threads.append(threading.Thread(target=arm, args=(robotList, )))
    behavior_threads.append(threading.Thread(target=timer))
    behavior_threads.append(threading.Thread(target=traveler))
    behavior_threads.append(threading.Thread(target=command, args=(paths, )))

    for thread in behavior_threads:
        thread.daemon = True
        thread.start()

    # gRobotDraw.start()
    # gFrame.mainloop()

    # paths.append()

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
