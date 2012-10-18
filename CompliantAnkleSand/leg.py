import numpy as np
import foot

class Leg:

	def __init__(self,lengths,loc,pm,footParams):
		'''Initializes new leg. Takes link lengths vector, leg position vector, and plus/minus 1 for kinematics calc'''
		self.L = lengths
		self.loc = loc
		self.pm = pm
		self.Foot = foot.Foot(self,footParams['rf'],footParams['angle'])

		#Vectors that hold joint angles, ang speeds, and ang accel
		self.theta = np.zeros(4)
		self.theta = self.calcAngles(0)
		self.omega = np.zeros(4)
		self.omega = self.calcAngSpeeds(0)
		self.alpha = np.zeros(4)
		self.alpha = self.calcAngAccels(0)
		
		#x, y position of joints. Rows are O, A, B, C, F respectively.
		self.jointPos = np.zeros((5,2))
		self.jointPos = self.calcPos()
		

	def calcAngles(self, theta1):
		'''Takes input link angle, performs 4 bar kinematics calcs and returns angles of all links'''

		self.theta[1] = theta1

		Alpha = 2*self.L[0]*self.L[3] - 2*self.L[1]*self.L[3]*np.cos(self.theta[1])
		Beta  = -2*self.L[1]*self.L[3]*np.sin(self.theta[1])
		Gamma = (self.L[1]**2 + self.L[3]**2 + self.L[0]**2 - self.L[2]**2 
					- 2*self.L[0]*self.L[1]*np.cos(self.theta[1]))
		Delta = np.arctan2(Beta,Alpha)

		self.theta[3] = Delta + self.pm*np.arccos(-1*Gamma/np.sqrt(Alpha**2 + Beta**2))
		self.theta[2] = (np.arctan2(self.L[3]*np.sin(self.theta[3]) 
							- self.L[1]*np.sin(self.theta[1]),self.L[0] 
							+ self.L[3]*np.cos(self.theta[3]) - self.L[1]*np.cos(self.theta[1])))
		return self.theta

	def calcAngSpeeds(self, omega1):
		'''Solves system of equations and returns angular speeds given input link speed omega1'''

		A = np.array([[-1*self.L[3]*np.sin(self.theta[3]),    self.L[2]*np.sin(self.theta[2])],
					     [self.L[3]*np.cos(self.theta[3]), -1*self.L[2]*np.cos(self.theta[2])]])
		b = np.array([[-1*self.L[1]*np.sin(self.theta[1])],
						 [self.L[1]*np.cos(self.theta[1])]]) * omega1
		omega32 = np.linalg.solve(A,b)
		self.omega[3] = omega32[0]
		self.omega[2] = omega32[1]
		self.omega[1] = omega1
		return self.omega
	
	def calcAngAccels(self, alpha1):
		'''Solves system of equations and returns angular accels given input link accel alpha1'''

		A  = np.array([[-1*self.L[3]*np.sin(self.theta[3]),    self.L[2]*np.sin(self.theta[2])],
					      [self.L[3]*np.cos(self.theta[3]), -1*self.L[2]*np.cos(self.theta[2])]])
		
		b1 = np.array([[-1*self.L[1]*np.sin(self.theta[1])],
					   [   self.L[1]*np.cos(self.theta[1])]])
		b2 = np.array([[-1*self.L[1]*np.cos(self.theta[1])],
					   [-1*self.L[1]*np.sin(self.theta[1])]])
		b3 = np.array([[-1*self.L[2]*np.cos(self.theta[2])],
					   [-1*self.L[2]*np.sin(self.theta[2])]])
		b4 = np.array([[   self.L[3]*np.cos(self.theta[3])],
					   [   self.L[3]*np.sin(self.theta[3])]])
		b = b1*alpha1 +b2*self.omega[1]**2 + b3*self.omega[2]**2 + b4*self.omega[3]**2

		alpha32 = np.linalg.solve(A,b)
		self.alpha[3] = alpha32[0]
		self.alpha[2] = alpha32[1]
		self.alpha[1] = alpha1
		return self.alpha
	
	def calcPos(self):
		'''Returns x,y coordinates of Joints O,A,B,C and Foot Connection of four bar'''
		O = self.loc
		self.jointPos[0,:] = O
		self.jointPos[1,:] = O + np.array([self.L[1] * np.cos(self.theta[1]), self.L[1] * np.sin(self.theta[1])]) #A
		self.jointPos[2,:] = np.array([self.L[0], 0]) + np.array([self.L[3]*np.cos(self.theta[3]), 
												 self.L[3]*np.sin(self.theta[3])]) #B
		self.jointPos[3,:] = O + np.array([self.L[0] * np.cos(self.theta[0]), self.L[0] * np.sin(self.theta[0])]) #C
		self.jointPos[4,:] = (O + np.array([self.L[1] * np.cos(self.theta[1]), self.L[1] * np.sin(self.theta[1])])
				+ np.array([self.L[4]*np.cos(self.theta[2] + np.pi), self.L[4]*np.sin(self.theta[2] + np.pi)])) #F
		return self.jointPos

	def calcForceTorque(self):
		'''Solves system of equations, and returns vector containing joint load
		and motor shaft torque given ground reaction force force, and moment
		x = [Fr1x Fr1y Fr2x Fr2y F2x F2y F4x F4y T]'''

		A = np.array([[0 0 0 0 1  0  1                               0                               0],
					  [0 0 0 0 0  1  0							     1							     0],		
					  [0 0 0 0 0  0  self.L[2]*np.sin(self.theta[2]) self.L[2]*np.cos(self.theta[2]) 0],
					  [0 0 1 0 -1 0  0							     0							     0],
					  [0 0 0 1 0  -1 0								 0								 1],
					  [1 0 0 0 0  0  -1								 0								 0],
					  [0 1 0 0 0  0  0                               -1								 0],
					  [0 0 0 0 0  0  self.L[3]*np.sin(self.theta[3]) self.L[3]*np.cos(self.theta[3]) 0]])

		footLoadx, loadLoady, footMoment = self.Foot.calcForce()
		b = np.array([[-1*footLoadx],
					  [-1*footLoady],
					  [-1*footMoment], 
					  [0], [0], [0], [0], [0], [0]])
	
		jointLoad = np.linalg.solve(A,b)
		return jointLoad


