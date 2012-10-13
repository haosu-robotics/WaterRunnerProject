import numpy as np
import foot

class Leg:

	def __init__(self,lengths,loc,pm):
		'''Initializes new leg. Takes link lengths vector, leg position vector, and plus/minus 1'''
		self.L = lengths
		self.loc = loc
		self.theta = np.empty(4)
		self.pm = pm

	def calcAngles(self, theta1):
		'''Takes input link angle, performs 4 bar kinematics calcs and returns angles of all links'''

		self.theta[1] = theta1

		alpha = 2*self.L[0]*self.L[3] - 2*self.L[1]*self.L[3]*np.cos(self.theta[1])
		beta  = -2*self.L[1]*self.L[3]*np.sin(self.theta[1])
		gamma = self.L[1]**2 + self.L[3]**2 + self.L[0]**2 - self.L[2]**2 - 2*self.L[0]*self.L[1]*np.cos(self.theta[1])
		delta = np.arctan2(beta,alpha)
		self.theta[3] = delta + self.pm*np.arccos(-1*gamma/np.sqrt(alpha**2 + beta**2))
		self.theta[2] = np.arctan2(self.L[3]*np.sin(self.theta[3]) - self.L[1]*np.sin(self.theta[1]),self.L[0] + self.L[3]*np.cos(self.theta[3]) - self.L[1]*np.cos(self.theta[1]))
		return self.theta

	def getPos(self):
		'''Returns x,y coordinates of Joints O,A,B,C and Foot Connection of four bar'''
		O = self.loc
		A = O + np.array([self.L[1] * np.cos(self.theta[1]), self.L[1] * np.sin(self.theta[1])])
		B = np.array([self.L[0], 0]) + np.array([self.L[3]*np.cos(self.theta[3]), self.L[3]*np.sin(self.theta[3])])
		C = O + np.array([self.L[0] * np.cos(self.theta[0]), self.L[0] * np.sin(self.theta[0])])
		F = O + np.array([self.L[1] * np.cos(self.theta[1]), self.L[1] * np.sin(self.theta[1])]) + np.array([self.L[4]*np.cos(self.theta[2] + np.pi), self.L[4]*np.sin(self.theta[2] + np.pi)])
		return O, A, B, C, F

	def calcAngSpeeds(self, omega1):
		'''Returns angular speeds omega2 and omega3 given input link speed omega1'''
		A = np.array([[-1*self.L[3]*np.sin(self.theta[3]), self.L[2]*np.cos(self.theta[2])],
					  [self.L[1]*np.cos(self.theta[3]), -1*self.L[2]*np.cos(self.theta[2])]])
		b = np.array([[-1*self.L[1]*np.sin(self.theta[1])],
						 [self.L[1]*np.sin(self.theta[1])]]) * omega1
		omega23 = np.linalg.solve(A,b)
		omega2 = omega23[1]
		omega3 = omega23[0]
		return omega2, omega3

	def calcFootSpeed(self, omega1):
		'''Returns x, y speed of foot given input link speed omega1'''
		omega2, omega3 = self.calcAngSpeeds(omega1)
		Fs =  (np.array([-1*self.L[1]*np.sin(self.theta[1]), self.L[1]*np.cos(self.cos(self.theta[1]))])*omega2
			 + np.array([-1*self.L[4]*np.sin(self.theta[2] + np.pi), self.L[4]*np.cos(self.theta[2] + np.pi)])*omega3)
		return Fs
			


