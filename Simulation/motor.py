import numpy as np

class Motor:

	def __init__(self,motorParams):
		self.pos = motorParams['initPos']
		self.speed = motorParams['initSpeed']
		self.accel = motorParams['initAccel']
		self.voltage = motorParams['voltage']
		self.J = motorParams['J']
		self.b = motorParams['b']
		self.Ke = motorParams['Ke']
		self.Kt = motorParams['Kt']
		self.R = motorParams['R']
		self.L = motorParams['L']
		
		self.state = np.array([[self.pos], 
		                      [self.speed], 
							  [0]])          #position speed, current
		self.dstate = np.array([[self.speed], 
							   [self.accel], 
							   [0]])         #speed accel, di/dt
		self.load = 0
	def update(self,load,voltage, timestep):
		self.load = load
		self.pos += timestep * self.speed
	'''
	
	def update(self,load,voltage, timestep):
		self.load = load
		self.dstate = (np.dot(np.array([[0, 1,                 0               ], 
					  				   [0,  -1*self.b/self.J,  self.Kt/self.J  ],
									   [0,  -1*self.Ke/self.L, -1*self.R/self.L]]), 
									   self.state) 
						+ np.array([[0], 
								   [load/self.J], 
								   [voltage/self.L]]))
		self.state += self.dstate*timestep

		self.pos = self.state[0]
		self.speed = self.state[1]
		self.accel = self.dstate[1]
		'''

