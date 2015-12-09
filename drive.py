import time

class Drive:
	def __init__(self, robotList):
		self.robotList = robotList
		self.robot = None
		self.stopQ = None

	def ready(self):
		return self.robot != None

	def waitForConn(self, stopQ):
		self.stopQ = stopQ
		while (not self.ready()) and self.stopQ.empty():
			if len(self.robotList) > 0:
				self.robot = self.robotList[0]
			time.sleep(0.2)

	def stop(self):
		self.drive(0, 0)

	def drive(self, left, right):
		if not self.stopQ.empty():
			return
		if not self.ready():
			print 'drive not ready'
			return
		self.robot.set_wheel(0, left)
		self.robot.set_wheel(1, right)
