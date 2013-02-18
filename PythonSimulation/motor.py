import numpy as np

class Motor:

	def __init__(self,motorParams, angleOffset):
		self.voltage = motorParams['voltage']
		self.ratio = motorParams['ratio']
		self.eff = motorParams['eff']
		
		self.pos = motorParams['initPos'] + angleOffset
		self.speed = motorParams['initSpeed']
		self.accel = motorParams['initAccel']

		self.J = motorParams['J']
		self.b = motorParams['b']
		self.Ke = motorParams['Ke']
		self.Kt = motorParams['Kt']
		self.R = motorParams['R']
		self.L = motorParams['L']
		
		self.state = np.array([[self.pos], 
		                      [self.speed], 
							  [0]])          #position, speed, current
		self.dstate = np.array([[self.speed], 
							   [self.accel], 
							   [0]])         #speed, accel, di/dt
		self.load = 0
	def update(self,load,voltage, timestep):
		self.load = load
		self.pos += timestep * self.speed
	'''
	def update(self,load,voltage, timestep):
		self.load = load
		self.dstate = (np.dot(np.array([[0, self.ratio,                 0               ], 
					  				    [0, -1*self.b*self.ratio/self.J,  self.Kt/self.J  ],
									    [0, -1*self.ratio*self.Ke/self.L, -1*self.R/self.L]]), 
									   self.state) 
						+ np.array([[0], 
								   [load/(self.J*self.eff*self.ratio)], 
								   [voltage/self.L]]))
		self.state += self.dstate*timestep

		self.pos = self.state[0]
		self.speed = self.state[1]
		self.accel = self.dstate[1]
		'''
