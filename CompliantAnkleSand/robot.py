import numpy as np
import leg

class Robot:
	
	def __init__(self,robotParams,legParams,footParams):
		'''initializes new robot with a leg and foot'''
		
		self.pos = robotParams['initPos']
		self.Leg1 = leg.Leg1[self,legParams,footParams)

	def calcAccel(self,theta1,omega1,alpha1)
		'''Calculate the acceleration of the robot given motor angle, speed, and acceleration.'''

				
	





