import numpy as np

class Foot():
	
	def __init__(self,initAngle,initPos,footParams,robotmass):
		'''Arguments:
		initAngle: vector containing initial angle, angular speed, angular accel of foot
		initPos:   matrix whos rows are x,y initial position, speed, accel of foot attach pt'''

		self.length = footParams['length']
		self.staticFrictionCoeff = footParams['staticFrictionCoeff']
		self.kinFrictionCoeff = footParams['kinFrictionCoeff']
		self.EI = footParams['EI']
		self.PRBMK = footParams['PRBMK']
		self.gamma = footParams['gamma']
		self.robotmass = robotmass

		#initialize angle position, speed, accel of foot
		self.theta = initAngle[0]
		self.thetaPRBM = 0.
		self.omega = initAngle[1]
		self.alpha = initAngle[2]

		#initialize cartesian position, speed, accel of foot
		self.pos = np.zeros((3,2)) #attatch point, psuedo-torsion spring loc, bottom
		self.pos[0,:] = initPos[0,:] 
		self.pos[1,:] = self.pos[0,:] - (1. - self.gamma)*self.length*np.array([np.cos(self.theta), np.sin(self.theta)])
		self.pos[2,:] = self.pos[1,:] - self.gamma*self.length*np.array([np.cos(self.theta), np.sin(self.theta)])
		self.speed = initPos[1,:]
		self.accel = initPos[2,:]

		#initalize forces
		self.loadx = 0.
		self.loady = 0.
		self.moment = 0.


	def update(self, pos, speed, accel, angle):
		#udpate state of foot
		self.calcForce()
		self.calcAngles(angle)
		self.calcPos(pos)
		self.calcSpeed(speed)
		self.calcAccel(accel)
	
	def calcAngles(self, angle):
		self.theta = angle
		magF = np.sqrt(self.loadx**2 + self.loady**2)
		phi = np.arctan2(self.loady,self.loadx)
		F_tan = magF * np.sin(phi - self.thetaPRBM - self.theta)
		
		self.thetaPRBM = ((F_tan*self.length**2.)/self.EI*self.PRBMK)*np.pi/180.
	
	def calcPos(self, pos):
		self.pos[0,:] = pos
		self.pos[1,:] = self.pos[0,:] - (1. - self.gamma)*self.length*np.array([np.cos(self.theta), np.sin(self.theta)]) 
		self.pos[2,:] = self.pos[1,:] - self.gamma*self.length*np.array([np.cos(self.theta - self.thetaPRBM), np.sin(self.theta - self.thetaPRBM)])
		return self.pos
		
	def calcSpeed(self, speed):
		self.speed = speed
		return speed

	def calcAccel(self, accel):
		self.accel = accel
		return accel

	def calcForce(self):
		'''Returns Ground Reaction Force and Moment'''
		yp = -1.*self.pos[2,1]
		ypdot = -1.*self.speed[1]
		if yp > 0:
			self.loady = 0.25e9 * (np.abs(yp)**3.)*(1.-0.1*np.abs(ypdot))
			if np.abs(self.robotmass*self.accel[0]) < self.staticFrictionCoeff*self.loady:
				self.loadx = self.robotmass*self.accel[0]
			else:
				self.loadx = -1*self.kinFrictionCoeff*np.abs(self.loady)*np.sign(self.speed[0])
			self.moment = -1*(self.loady*(self.pos[2,1]- self.pos[0,1])) + self.loadx*(self.pos[2,0] - self.pos[0,0])
			#print self.loady, yp, ypdot, self.thetaPRBM, self.pos[2,1]
		else:
			self.loady = 0.
			self.loadx = 0.
			self.moment = 0.
		return self.loadx, self.loady, self.moment
