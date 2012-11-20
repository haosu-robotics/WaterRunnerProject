import numpy as np

class Robot:
		
	def __init__(self,robotParams,worldParams,legs,motors):
		'''initializes new robot with a legs
		legParams: dict containing robot dimensions, mass, moment of inertia, center of mass, intial position, speed, accel
		leg: list of legs'''
		
		#initialize arrays holding properties
		self.mass = robotParams['mass']
		self.pos = np.array(2)
		self.pos = np.array(robotParams['initPos'])
		self.speed = np.array(2)
		self.speed = np.array(robotParams['initSpeed'])
		self.accel = np.array(2)
		self.accel = np.array(robotParams['initAccel'])
		self.Force = np.zeros(2)

		self.grav = worldParams['gravity']
		self.timeStep = worldParams['timeStep']
		self.time = 0.
		
		#Adam's bashforth history
		self.accel3 = None
		self.accel2 = None
		self.accel1 = None
		self.speed3 = None
		self.speed2 = None
		self.speed1 = None

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
		
		return 

	def calcAccel(self):
		'''sum forces and divide by mass to find x,y accel'''
		#Update last accel history for adam's bashforth
		self.accel3 = self.accel2
		self.accel2 = self.accel1
		self.accel1 = self.accel
		
		self.Force = np.array([self.calcAirDrag(self.speed[0]), self.mass*self.grav])
		for leg in self.legs:
			self.Force += leg.robotLoad[0:2]
	
		self.accel = self.Force/self.mass
		#self.accel[0] = 0.
		return self.accel

	def calcAirDrag(self, xspeed):
		density = 1.225
		A = 0.1*0.03
		dragCoeff = 2.0
		drag = -1*0.5*density*xspeed*np.abs(xspeed)*dragCoeff*A
		return drag

	def calcSpeed(self):
		self.speed3 = self.speed2
		self.speed2 = self.speed1
		self.speed1 = self.speed
		#perform euler's for first 4 iterations then adams-bashforth
		if self.accel3 == None:
			self.speed += self.timeStep*self.accel
		else:
			self.speed += (self.timeStep/24.) * (55.*self.accel - 59.*self.accel1 + 37.*self.accel2 - 9.*self.accel3)
		#self.speed[0] = 0
		return self.speed
	
	def calcPos(self):
		#perform euler's for first 4 itereations then adams-bashforth
		if self.speed3 == None:
			self.pos += self.speed*self.timeStep + (self.accel/2)*self.timeStep**2
		else:
			self.pos += (self.timeStep/24.) * (55.*self.speed - 59.*self.speed1 + 37.*self.speed2 - 9.*self.speed3)
		#self.pos[0] = 0
		return self.pos
