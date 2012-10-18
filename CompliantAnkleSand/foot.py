import numpy as np

class Foot():
	
	def __init__(self,Leg,radius,angle):
		self.Leg = Leg
		self.radius = radius
		self.angle = angle
	
	def calcEndPos():
		return self.Leg.jointPos[4]

	def calcSpeed(self):
		'''Returns x, y speed of foot given omega1, the speed of leg link 1'''

		Fs =  (np.array([[-1*self.Leg.L[1]*np.sin(self.Leg.theta[1])], 
		                 [	 self.Leg.L[1]*np.cos(self.Leg.theta[1])]])*self.Leg.omega[1]
			 + np.array([[-1*self.Leg.L[4]*np.sin(self.Leg.theta[2] + np.pi)], 
						 [   self.Leg.L[4]*np.cos(self.Leg.theta[2] + np.pi)]])*self.Leg.omega[3])
		return Fs

	#def calcForce(self):
		'''Calculates and returns ground reaction force'''
		



