import time

class Pen:
	def __init__(self, robotList, moveDuration = 3.0):
		self.robotList = robotList
		self.robot = None
		self.sleep_time = moveDuration

	def ready(self):
		return self.robot != None

	def waitForConn(self, stopQ):
		while (not self.ready()) and stopQ.empty():
			if len(self.robotList) > 1:
				self.robot = self.robotList[1]
			time.sleep(0.2)

	def lift(self):
		if not self.ready():
			print 'pen not ready'
			return
		self.robot.set_wheel(0, 10)
		self.robot.set_wheel(1, 10)
		time.sleep(self.sleep_time + 1.0)
		self.robot.set_wheel(0, 0)
		self.robot.set_wheel(1, 0)

	def lower(self):
		if not self.ready():
			print 'pen not ready'
			return
		self.robot.set_wheel(0, -10)
		self.robot.set_wheel(1, -10)
		time.sleep(self.sleep_time - 1.0)
		self.robot.set_wheel(0, 0)
		self.robot.set_wheel(1, 0)
