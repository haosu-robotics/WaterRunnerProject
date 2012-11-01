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
	
	def update(self, angle, pos):
		'''Updates state of foot by calling methods to calculate joint angular position, speed, and accel, 
		and cartesian posiiton, speed and accels.'''
		#update angular state
		self.calcAngle(angle[0])

		#update cartesian state
		self.calcPos(pos[0,:])
		self.calcSpeed(pos[1,:])
		self.calcAccel(pos[2,:])
	
	def calcAngle(angle):
		'''calculates angle of foot'''
		self.theta = angle
		return angle

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
		Fw
		self.moment = 0.
		return self.loadx, self.loady, self.moment

	def Drag(s
	
	
	def a(s):
		vs = 
		
		return np.dot(self.speed,np.array([np.cos(self.theta[0]), np.sin(self.theta[1])]))
