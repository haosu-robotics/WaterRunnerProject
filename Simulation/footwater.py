import numpy as np
import scipy.integrate as integrate

class Foot():
	
	def __init__(self,initAngle,initPos,footParams,worldParams,robotMass):
		'''Arguments:
		initAngle: vector containing initial angle, angular speed, angular accel of foot
		initPos:   matrix whos rows are x,y initial position, speed, accel of foot attach pt'''
		
		self.radius = footParams['radius']
		self.radiusUp = footParams['radiusUp']
		self.density = footParams['density']
		self.dragCoeff = footParams['dragCoeff']
		self.gravity = worldParams['gravity']
		self.timeStep = worldParams['timeStep']
		self.robotMass = robotMass

		#initialize angle position, speed, accel of foot
		self.theta = initAngle[0]
		self.omega = initAngle[1]
		self.alpha = initAngle[2]

		#initialize cartesian position, speed, accel of foot
		self.pos   = initPos[0,:] 
		self.speed = initPos[1,:]
		self.accel = initPos[2,:]

		#initalize forces
		self.loadx = 0.
		self.loady = 0.
		self.moment = 0.

	def update(self, angle, pos, timestep):
		'''Updates state of foot by calling methods to calculate joint angular position, speed, and accel, 
		and cartesian posiiton, speed and accels. Updates force/torque on leg.'''

		self.calcAngPos(angle[0])
		self.calcAngSpeed(angle[1])
		self.calcPos(pos[0])
		self.calcSpeed(pos[1])
		self.calcAccel(pos[2])
		self.calcForce()

	def calcAngSpeed(self, omega):
		self.omega = omega
	
	def calcAngPos(self, theta):
		self.theta = theta
		
	def calcPos(self, pos):
		self.pos = pos
		
	def calcSpeed(self, speed):
		self.speed = speed

	def calcAccel(self, accel):
		self.accel = accel

	def calcForce(self):
		normalVect = np.array([np.sin(self.theta), -1.*np.cos(self.theta)])
		normalVelComp = np.dot(normalVect, self.speed)
		
		#unfolded state
		if normalVelComp >= 0:
			if  self.y_bf(self.radius) > 0:
				self.loadx = 0.
				self.loady = 0.
				self.moment = 0.		
				return
			else:	
				if self.theta >= 0:
					self.percentSub = -1.*self.y_bf(self.radius)/(-2.*self.radius*np.sin(self.theta + np.pi))
				else:
					self.percentSub = -1.*self.y_bf(self.radiusUp)/(-2.*self.radiusUp*np.sin(self.theta))
				if self.percentSub > 1:
					self.percentSub = 1.
				normal1 = np.array([np.sin(self.theta), -1.*np.cos(self.theta)])
				force1, _ = integrate.quad(self.drag_s,-1*self.radius, -1.*self.radius+2.*self.percentSub*self.radius, args =(self.radius, normal1))
				force2 = 0
		#folded state
		else:
			if  self.y_bf(self.radiusUp) > 0:
				self.loadx = 0.
				self.loady = 0.
				self.moment = 0.
				return
			else:
				if self.theta >= 0:
					self.percentSub = -1.*self.y_bf(self.radiusUp)/(-2.*self.radiusUp*np.sin(self.theta + np.pi))
				else:
					self.percentSub = -1.*self.y_bf(self.radiusUp)/(-2.*self.radiusUp*np.sin(self.theta))
				if self.percentSub > 1:
					self.percentSub = 1.
				normal1 = np.array([np.sin(self.theta), -1.*np.cos(self.theta)])
				force1, _ = integrate.quad(self.drag_s,-1*self.radiusUp, -1.*self.radiusUp+2.*self.percentSub*self.radiusUp, args = (self.radiusUp, normal1))
				
				force2 = 0
				normal2 = np.array([-1*np.cos(self.theta), -1.*np.sin(self.theta)])
				force2, _ = integrate.quad(self.drag_s,-1*self.radius, -1.*self.radius+2.*self.percentSub*self.radius, args = (self.radius, normal2))
		self.loadx = -1.*force1*np.sin(self.theta) + force2 * np.cos(self.theta)
		self.loady =     force1*np.cos(self.theta) + force2 * np.sin(self.theta)
		self.moment = 0.
		#print  'Fx: ', self.loadx, 'Fy: ', self.loady, 'normal: ', normalVelComp, 'vel: ', self.speed, 'y: ', self.y_bf(self.radius), 'theta: ',self.theta*180./np.pi
	
	def y_bf(self, radius):
		if self.theta >= 0:
			return self.pos[1] + radius*np.sin(self.theta + np.pi)
		else:
			return self.pos[1] + radius*np.sin(self.theta)

	def velocity_s(self, s, radius):
		v_bf = self.speed + radius*np.array([-1.*np.sin(self.theta + np.pi), np.cos(self.theta + np.pi)]) * self.omega
		v_tf = self.speed - radius*np.array([-1.*np.sin(self.theta + np.pi), np.cos(self.theta + np.pi)]) * self.omega
		vel = ((v_tf - v_bf)/(2.*radius))*s + (v_tf + v_bf)/2.
		return vel

	def normal_s(self, s, radius, normal):
		a_s = np.dot(normal,self.velocity_s(s, radius))
		return a_s

	def depth_s(self, s, radius):
		depth = -1.*self.y_bf(radius)*(1. - (s + radius)/(2.*self.percentSub*radius))
		return depth

	def drag_s(self, s, radius, normal):
		drag = self.dragCoeff*self.density*np.sqrt(radius**2. - s**2)*(-2.*self.gravity*self.depth_s(s, radius) + self.normal_s(s, radius, normal)*np.abs(self.normal_s(s, radius, normal)))
		return drag
