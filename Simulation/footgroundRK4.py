import numpy as np
import pdb

class Foot():
	
	def __init__(self,initAngle,initPos,footParams,worldParams,robotMass):
		'''Arguments:
		initAngle: vector containing initial angle, angular speed, angular accel of foot
		initPos:   matrix whos rows are x,y initial position, speed, accel of foot attach pt'''

		self.staticFrictionCoeff = footParams['staticFrictionCoeff']
		self.kinFrictionCoeff = footParams['kinFrictionCoeff']
		
		self.length = footParams['length']
		self.E = footParams['E']*10.**6.
		self.PRBMK = footParams['PRBMK']
		self.gamma = footParams['gamma']
		self.inertia = (footParams['width']*footParams['thickness']**3.)/12.
		self.inertiaEff = .3e-6
		#self.k = (self.gamma*self.PRBMK*self.E*self.inertia/self.length)*180./np.pi
		self.k = footParams['k']
		self.b = footParams['damping']*10.**-5.

		self.gravity = worldParams['gravity']
		self.timeStepFine = worldParams['timeStepFine']
		self.robotMass = robotMass

		#initialize angle position, speed, accel of foot
		self.theta = initAngle[0]
		self.omega = initAngle[1]
		self.alpha = initAngle[2]
		self.thetaPRBM = 0.
		self.omegaPRBM = 0.
		self.alphaPRBM = 0.

		#initialize cartesian position, speed, accel of foot
		self.pos = np.zeros((3,2), dtype = float) #attatch point, psuedo-torsion spring loc, bottom
		self.pos[0,:] = initPos[0,:] 
		self.pos[1,:] = self.pos[0,:] - (1. - self.gamma)*self.length*np.array([np.cos(self.theta), np.sin(self.theta)])
		self.pos[2,:] = self.pos[1,:] - self.gamma*self.length*np.array([np.cos(self.theta), np.sin(self.theta)])
		self.speed = np.zeros((3,2), dtype = float)
		self.speed[0,:] = initPos[1,:]
		self.speed[1,:] = initPos[1,:]
		self.speed[2,:] = initPos[1,:]
		
		self.accel = np.zeros((3,2), dtype = float)
		self.accel[0,:] = initPos[2,:]
		self.accel[1,:] = initPos[2,:]
		self.accel[2,:] = initPos[2,:]


		#initalize forces
		self.loadx = 0.
		self.loady = 0.
		self.moment = 0.
		self.bendingMoment = 0.

	def update(self, angle, pos, timeStep):
		'''Updates state of foot by calling methods to calculate joint angular position, speed, and accel, 
		and cartesian posiiton, speed and accels. Updates force/torque on leg.'''
		
		self.theta = angle[0]
		self.omega = angle[1]
		self.alpha = angle[2]
		
		self.calcPos(pos[0])
		self.calcSpeed(pos[1])
		self.calcAccel(pos[2])

		totalLoadx = 0
		totalLoady = 0
		totalMoment = 0
		self.timeStep = self.timeStepFine
		timeStepRatio = timeStep/self.timeStepFine
		state = np.array([self.pos[2,0], self.pos[2,1], self.speed[2,0], self.speed[2,1], self.thetaPRBM, self.omegaPRBM])
		#print state
		for i in np.arange(timeStepRatio):
			#print i, timeStepRatio
			#pdb.set_trace()
			_, _, _, fineBendingMoment1 = self.calcForce(self.pos[2,:], self.speed[2,:], self.accel[2,:])	
			state1 = state
			k1 = self.timeStep * self.calcStateDeriv(state1, fineBendingMoment1)
			#print 'state1: ', state1
			#print 'k1: ', k1
			#pdb.set_trace()
			state2 = state1 + 0.5 * k1
			_, _, _, fineBendingMoment2 = self.calcForce(state2[[0,1]], state2[[2,3]], self.accel[2,:])
			k2 = self.timeStep * self.calcStateDeriv(state2, fineBendingMoment2)
			#print 'state2: ', state2
			#print 'k2: ', k2

			#pdb.set_trace()
			state3 = state1 + 0.5 * k2
			_, _, _, fineBendingMoment3 = self.calcForce(state3[[0,1]], state3[[2,3]], self.accel[2,:])		
			k3 = self.timeStep * self.calcStateDeriv(state3, fineBendingMoment3)
			#print 'state3: ', state3
			#print 'k3: ', k3

			#pdb.set_trace()
			state4 = state1 + k3
			_, _, _, fineBendingMoment4 = self.calcForce(state4[[0,1]], state4[[2,3]], self.accel[2,:])	
			k4 = self.timeStep * self.calcStateDeriv(state4, fineBendingMoment4)
			#print 'state4: ', state4
			#print 'k4: ', k4

			#pdb.set_trace()
			state += (1./6.) * (k1 + 2.*k2 + 2.*k3 + k4)		
			fineLoadx, fineLoady, fineMoment, fineBendingMoment = self.calcForce(state[[0,1]], state[[2,3]], self.accel[2,:])
			#print 'state: ', state
			totalLoadx += fineLoadx
			totalLoady += fineLoady
			totalMoment += fineMoment

		dstate = self.calcStateDeriv(state, fineBendingMoment)
		self.thetaPRBM = state[4]
		self.omegaPRBM = state[5]
		self.alphaPRBM = dstate[5]

		self.loadx = totalLoadx/timeStepRatio
		self.loady = totalLoady/timeStepRatio
		self.moment = totalMoment/timeStepRatio
		

	def calcStateDeriv(self, state, bendingMoment):
		#state = x, y, x',y', theta, omega
		x = state[0]
		y = state[1]
		thetaPRBM = state[4]
		omegaPRBM = state[5]

		#dstate
		alphaPRBM = (-1*self.b*omegaPRBM - self.k*thetaPRBM + bendingMoment)/self.inertiaEff
		vx, vy = self.speed[1,:] + self.gamma * self.length * np.array([-1 * np.sin(self.theta + np.pi - thetaPRBM), np.cos(self.theta + np.pi - thetaPRBM)])*(self.omega - omegaPRBM)
		ax, ay = (self.accel[1,:]+ self.gamma * self.length * np.array([-1 * np.cos(self.theta + np.pi - thetaPRBM), -1* np.sin(self.theta + np.pi - thetaPRBM)])*(self.omega - omegaPRBM)**2 
					+ self.gamma * self.length * np.array([-1 * np.sin(self.theta + np.pi - thetaPRBM), np.cos(self.theta + np.pi - thetaPRBM)])*(self.alpha - self.alphaPRBM))
		dstate = np.array([vx, vy, ax, ay, omegaPRBM, alphaPRBM])

		#pdb.set_trace()
		return dstate

	def calcPos(self, pos):
		self.pos[0,:] = pos
		self.pos[1,:] = self.pos[0,:] - (1. - self.gamma) * self.length * np.array([np.cos(self.theta), np.sin(self.theta)]) 
		self.pos[2,:] = self.pos[1,:] - self.gamma * self.length * np.array([np.cos(self.theta - self.thetaPRBM), np.sin(self.theta - self.thetaPRBM)])

	def calcSpeed(self, speed):
		self.speed[0,:] = speed
		self.speed[1,:] = self.speed[0,:] + (1 - self.gamma) * self.length * np.array([-1* np.sin(self.theta + np.pi), np.cos(self.theta + np.pi)]) * self.omega
		self.speed[2,:] = self.speed[1,:] + self.gamma * self.length *np.array([-1 * np.sin(self.theta + np.pi - self.thetaPRBM), np.cos(self.theta + np.pi - self.thetaPRBM)])*(self.omega - self.omegaPRBM)

	def calcAccel(self, accel):
		self.accel[0,:] = accel
		self.accel[1,:] = (self.accel[0,:] + (1 - self.gamma) * self.length * np.array([-1 * np.cos(self.theta + np.pi), 
																					    -1 * np.sin(self.theta + np.pi)]) * self.omega**2 
						  + (1 - self.gamma) * self.length * np.array([-1* np.sin(self.theta + np.pi), 
																		   np.cos(self.theta + np.pi)]) * self.alpha)
		self.accel[2,:] = (self.accel[1,:]+ self.gamma * self.length * np.array([-1 * np.cos(self.theta + np.pi - self.thetaPRBM), 
							   													  -1 * np.sin(self.theta + np.pi - self.thetaPRBM)])*(self.omega - self.omegaPRBM)**2 
						  + self.gamma * self.length * np.array([-1 * np.sin(self.theta + np.pi - self.thetaPRBM), 
															          np.cos(self.theta + np.pi - self.thetaPRBM)])*(self.alpha - self.alphaPRBM))

	def calcForce(self, pos, speed, accel):
		'''Returns Ground Reaction Force and Moment'''
		yp = -1.*pos[1]
		ypdot = -1.*speed[1]

		if yp > 0:
			if ypdot > 5:
				ypdot = 5.
			elif ypdot < -2:
				ypdot = -2.
			
			if ypdot >=0:
				loady = 0.25e9 * (np.abs(yp)**3.)*(1.- 0.1*ypdot)
			else:
				loady = 0.25e9 * (np.abs(yp)**3.)*(1.+ ypdot)
			
			#self.speed = np.zeros((3,2))
			if np.abs(self.robotMass*accel[0]) < self.staticFrictionCoeff*loady:
				loadx = -1*self.robotMass*accel[0]
			else:
				loadx = -1*self.kinFrictionCoeff*np.abs(loady)*np.sign(speed[0])
			moment = -1.*loady*(pos[0]- self.pos[0,0]) + loadx*(pos[1] - self.pos[1,1])
			bendingMoment = moment - loady*(self.pos[0,0]- self.pos[1,0]) + loadx*(self.pos[0,1] - self.pos[1,1])
		else:
			loady = 0.
			loadx = 0.
			moment = 0.
			bendingMoment = 0.
		#pdb.set_trace()
		return loadx, loady, moment, bendingMoment

