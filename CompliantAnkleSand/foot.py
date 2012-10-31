import numpy as np

class Foot():
	
	def __init__(self,initAngle,initPos,footParams):
		'''Arguments:
		initAngle: vector containing initial angle, angular speed, angular accel of foot
		initPos:   matrix whos rows are x,y initial position, speed, accel of foot attach pt'''

		self.radius = footParams['radius']
		self.angle = footParams['angle']
		
		#initialize angle position, speed, accel of foot
		self.theta = initAngle[0]
		self.omega = initAngle[1]
		self.alpha = initAngle[2]

		#initialize cartesian position, speed, accel of foot
		self.pos = initPos[0,:]
		self.speed = initPos[1,:]
		self.accel = initPos[2,:]
	
	def update(self, pos, speed, accel):
		#udpate state of foot
		self.calcPos(pos)
		self.calcSpeed(speed)
		self.calcAccel(accel)
		self.calcForce()

	def calcPos(self, pos):
		self.pos = pos
		return pos
		
	def calcSpeed(self, speed):
		self.speed = speed
		return self.speed

	def calcAccel(self, accel):
		self.accel = accel
		return accel

	def calcForce(self):
		'''Returns Ground Reaction Force and Moment'''
		self.loadx = 0.
		yp = -1*self.pos[1]
		ypdot = -1*self.speed[1]
		if yp > 0:
			self.loady = 0.25e9 * (np.abs(yp)**3)/ypdot
			print yp, ypdot, self.loady
		else:
			self.loady = 0
		self.moment = 0.
		return self.loadx, self.loady, self.moment
