import numpy as np
import foot

class Leg:

	def __init__(self,lengths,loc,pm):
		self.L = lengths
		self.x = loc[0]
		self.y = loc[1]
		self.theta = np.empty(4)
		self.theta[0] = loc[3]

	def calcAngles(self, theta1):
		'''Takes input link angle, performs 4 bar kinematics calcs and returns angles of all links'''

		self.theta[1] = theta1

		alpha = self.L[0]**2 +2*self.L[0]*self.L[3]*np.cos(self.theta[1])
		beta  = -2*self.L[1]*self.L[3]*np.sin(self.theta[1])
		gamma = self.L[1]**2 + self.L[3]**2 + self.L[0]**2 - self.L[2]**2 - 2*self.L[0]*self.L[1]*np.cos(self.theta[1])
		delta = np.arctan2(beta/alpha)
		
		self.theta[2] = np.arctan2(self.L[3]*np.sin(self.theta[3]) - self.L[1]*np.sin(self.theta[1]),self.L[0] + self.L[3]*np.cos(self.theta[3]) - self.L[1]*np.sin(self.theta[1]))
		self.theta[3] = delta + pm*np.arccos(-1*gamma/np.sqrt(alpha**2 + beta**2))
	
		return self.theta
