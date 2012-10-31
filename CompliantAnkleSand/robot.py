import numpy as np

class Robot:
		
	def __init__(self,robotParams,worldParams,legs,motors):
		'''initializes new robot with a legs
		legParams: dict containing robot dimensions, mass, moment of inertia, center of mass, intial position, speed, accel
		leg: list of legs'''
		
		#initialize arrays holding properties
		self.mass = robotParams['mass']
		self.pos = np.array(2)
		self.pos = robotParams['initPos']
		self.speed = np.array(2)
		self.speed = robotParams['initSpeed']
		self.accel = np.array(2)
		self.accel = robotParams['initAccel']
		self.Force = np.zeros(2)

		self.grav = worldParams['gravity']
		self.timeStep = worldParams['timeStep']
		self.time = 0.

		#add legs
		self.legs = []
		for leg in legs:
			self.legs.append(leg)	
		
		#add motors
		self.motors= []
		for motor in motors:
			self.motors.append(motor)

		#motor voltage
		self.motorVoltage = 5.

	def update(self):
		'''Updates state of legs to calculate forces on robot then updates accel speed, position of robot'''

		#calculate update leg/motor pairs
		for leg, motor in zip(self.legs,self.motors):
			leg.update([motor.pos, motor.speed, motor.accel], np.array([[self.pos], [self.speed], [self.accel]]))
			torque = leg.robotLoad[2]
			motor.update(torque,self.motorVoltage)
			
		self.calcAccel()
		self.calcSpeed()
		self.calcPos()
		
		self.time += self.timeStep

	def calcAccel(self):
		'''sum forces and divide by mass to find x,y accel'''

		self.Force = np.array([0, self.mass*self.grav])
		for leg in self.legs:
			self.Force += leg.robotLoad[0:2]
	
		self.accel = self.Force/self.mass
		return self.accel

	def calcSpeed(self):
		self.speed += self.timeStep*self.accel
		return self.speed
	
	def calcPos(self):
		self.pos += self.speed*self.timeStep + (self.accel/2)*self.timeStep**2
		return self.pos
