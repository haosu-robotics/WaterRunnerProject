import numpy as np

class Robot:
		
	def __init__(self,robotParams,worldParams,legs,motors):
		'''initializes new robot with a legs
		legParams: dict containing robot dimensions, mass, moment of inertia, center of mass, intial position, speed, accel
		leg: list of legs'''
		
		#initialize arrays holding properties
		self.mass = robotParams['mass']
		self.inertia = robotParams['Ipitch']
		self.COM = robotParams['lCOM']
		self.pos = np.array(2)
		self.pos = np.array(robotParams['initPos'])
		self.speed = np.array(2)
		self.speed = np.array(robotParams['initSpeed'])
		self.accel = np.array(2)
		self.accel = np.array(robotParams['initAccel'])
		self.Force = np.zeros(2)

		self.theta = 0.
		self.omega = 0.
		self.alpha = 0.
		self.Torque = 0.
		self.grav = worldParams['gravity']
		self.time = 0.
		
		#Adam's bashforth history
		self.accel3 = None
		self.accel2 = None
		self.accel1 = None
		self.speed3 = None
		self.speed2 = None
		self.speed1 = None
		
		self.alpha3 = None
		self.alpha2 = None
		self.alpha1 = None
		self.omega3 = None
		self.omega2 = None
		self.omega1 = None

		#add legs
		self.legs = []
		for leg in legs:
			self.legs.append(leg)	
		
		#add motors
		self.motors= []
		for motor in motors:
			self.motors.append(motor)

	def update(self, timestep):
		'''Updates state of legs to calculate forces on robot then updates accel speed, position of robot'''
		self.timeStep = timestep
		#calculate update leg/motor pairs
		i = 0
		for leg, motor in zip(self.legs,self.motors):
			torque = leg.robotLoad[-1]
			motor.update(torque, motor.voltage,timestep)		
			
			position = self.pos + np.array([leg.initPos[0,0] * np.cos(self.theta), leg.initPos[0,0] * np.sin(self.theta)])
			speed = self.speed + np.array([-1*leg.initPos[0,0] * np.sin(self.theta)*self.omega, leg.initPos[0,0] * np.cos(self.theta) * self.omega])
			accel = self.accel + np.array([-1*leg.initPos[0,0] * np.cos(self.theta)*self.omega**2, -1*leg.initPos[0,0] * np.sin(self.theta)*self.omega**2]) +  np.array([-1*leg.initPos[0,0] * np.sin(self.theta)*self.alpha, leg.initPos[0,0] * np.cos(self.theta)*self.alpha])

			leg.update(np.array([self.theta, self.omega, self.alpha]), np.array([motor.pos, motor.speed, motor.accel]) + np.array([self.theta, self.omega, self.alpha]), np.array([[position], [speed], [accel]]), timestep)
			i += 1
			
		self.calcAccel()
		self.calcSpeed()
		self.calcPos()
		self.calcAlpha()
		self.calcOmega()
		self.calcTheta()
		self.time += self.timeStep
		
		return 

	def calcAccel(self):
		'''sum forces and divide by mass to find x,y accel'''
		#Update last accel history for adam's bashforth
		self.accel3 = self.accel2
		self.accel2 = self.accel1
		self.accel1 = self.accel
		
		self.Force = np.array([0., self.mass*self.grav])
		for leg in self.legs:
			self.Force[0] += leg.Foot.loadx #leg.robotLoad[0] + leg.robotLoad[2]
			self.Force[1] += leg.Foot.loady #leg.robotLoad[1] + leg.robotLoad[3]
	
		self.accel = self.Force/self.mass
		#self.accel = np.zeros(2)
		return self.accel

	def calcSpeed(self):
		self.speed3 = self.speed2
		self.speed2 = self.speed1
		self.speed1 = self.speed
		#perform euler's for first 4 iterations then adams-bashforth
		if self.accel3 == None:
			self.speed += self.timeStep*self.accel
		else:
			self.speed += (self.timeStep/24.) * (55.*self.accel - 59.*self.accel1 + 37.*self.accel2 - 9.*self.accel3)
		#self.speed = np.zeros(2)
		return self.speed
	
	def calcPos(self):
		#perform euler's for first 4 itereations then adams-bashforth
		if self.speed3 == None:
			self.pos += self.speed*self.timeStep + (self.accel/2)*self.timeStep**2
		else:
			self.pos += (self.timeStep/24.) * (55.*self.speed - 59.*self.speed1 + 37.*self.speed2 - 9.*self.speed3)
		return self.pos

	def calcAlpha(self):
		'''sum torques and divide by moment of inertia to find angular accel'''
		#Update last accel history for adam's bashforth
		self.alpha3 = self.alpha2
		self.alpha2 = self.alpha1
		self.alpha1 = self.alpha
		
		self.Torque = 0.
		for leg in self.legs:
			#print 'footloadx: ', leg.Foot.loadx, ' robotLoad: ', leg.robotLoad[0] + leg.robotLoad[2]
			#print 'footloady: ', leg.Foot.loady, ' robotLoad: ', leg.robotLoad[1] + leg.robotLoad[3]
			#print leg.jointLoad
			Torque1 = leg.robotLoad[-1]
			Torque2 = leg.robotLoad[0]*(self.pos[1] - leg.jointPos[0,1]) - leg.robotLoad[1]*(self.pos[0] - leg.jointPos[0,0])
			Torque3 = leg.robotLoad[2]*(self.pos[1] - leg.jointPos[3,1]) - leg.robotLoad[3]*(self.pos[0] - leg.jointPos[3,0])
			self.Torque += Torque1 + Torque2 + Torque3
		self.alpha = self.Torque/self.inertia
		self.alpha = 0
		return self.alpha

	def calcOmega(self):
		self.omega3 = self.omega2
		self.omega2 = self.omega1
		self.omega1 = self.omega
		#perform euler's for first 4 iterations then adams-bashforth
		if self.alpha3 == None:
			self.omega += self.timeStep*self.alpha
		else:
			self.omega += (self.timeStep/24.) * (55.*self.alpha - 59.*self.alpha1 + 37.*self.alpha2 - 9.*self.alpha3)
		self.omega = 0
		return self.omega
	
	def calcTheta(self):
		#perform euler's for first 4 itereations then adams-bashforth
		if self.omega3 == None:
			self.pos += self.omega*self.timeStep + (self.alpha/2)*self.timeStep**2
		else:
			self.theta += (self.timeStep/24.) * (55.*self.omega - 59.*self.omega1 + 37.*self.omega2 - 9.*self.omega3)
		return self.theta
