import numpy

class Motor:

	def __init__(self,motorParams,worldParams):
		self.pos = motorParams['initPos']
		self.speed = motorParams['initSpeed']
		self.accel = motorParams['initAccel']
		self.load = 0
		self.voltage = 0
		
		self.timeStep = worldParams['timeStep']

	def update(self,load,voltage):
		self.voltage = voltage
		self.load = load
		self.calcSpeed(load,voltage)
		self.calcPos()

	def calcSpeed(self,load,voltage):
		return self.speed

	def calcPos(self):
		self.pos += self.speed*self.timeStep
		
