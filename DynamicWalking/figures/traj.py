import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

rc('font', family='serif')

def calcAngles(theta1):
	'''Takes input link angle, performs 4 bar kinematics calcs and returns angles of all links'''
	theta = np.zeros(4)
	theta[0] = 0
	theta[1] = theta1

	Alpha = 2.*L[0]*L[3] * np.cos(theta[0]) - 2.*L[1]*L[3]*np.cos(theta[1])
	Beta  = 2.*L[0]*L[3] * np.sin(theta[0]) - 2.*L[1]*L[3]*np.sin(theta[1])
	Gamma = (L[0]**2 + L[1]**2 + L[3]**2 - L[2]**2 - 2.*L[0]*L[1]*np.cos(theta[0]) * np.cos(theta[1])
																	   - 2.*L[0]*L[1]*np.sin(theta[0]) * np.sin(theta[1]))
	Delta = np.arctan2(Beta,Alpha)

	theta[3] = Delta + np.arccos(-1*Gamma/np.sqrt(Alpha**2 + Beta**2))
	theta[2] = (np.arctan2(L[0]*np.sin(theta[0]) + L[3]*np.sin(theta[3]) - L[1]*np.sin(theta[1]),
						L[0]*np.cos(theta[0]) + L[3]*np.cos(theta[3]) - L[1]*np.cos(theta[1])))
	return theta

def calcPos(theta):
	Pos = np.zeros((5,2))
	Opos = np.zeros((1,2))
	'''Returns x,y coordinates of Joints O,A,B,C and Foot Connection of four bar given position of point O'''
	Pos[0,:] =  Opos
	Pos[1,:] =  Opos + np.array([L[1]*np.cos(theta[1]), L[1]*np.sin(theta[1])]) #A
	Pos[2,:] = (Opos + np.array([L[0]*np.cos(theta[0]), L[0]*np.sin(theta[0])]) 
							   + np.array([L[3]*np.cos(theta[3]), L[3]*np.sin(theta[3])])) #B
	Pos[3,:] =  Opos + np.array([L[0]*np.cos(theta[0]), L[0]*np.sin(theta[0])]) #C
	Pos[4,:] = (Opos + np.array([L[1]*np.cos(theta[1]), L[1]*np.sin(theta[1])])
							   + np.array([L[4]*np.cos(theta[2] + np.pi), L[4]*np.sin(theta[2] + np.pi)]))   #F
	return Pos

def calcAngSpeeds(theta):
	'''Solves system of equations and returns angular speeds given input link speed omega1'''
	omega = np.zeros(4)
	omega[1] = 70.
	omega[0] = 0.

	A = np.array([[-1*L[3]*np.sin(theta[3]),    L[2]*np.sin(theta[2])],
					 [L[3]*np.cos(theta[3]), -1*L[2]*np.cos(theta[2])]])
	b = ( np.array([[-1*L[1]*np.sin(theta[1])],
					   [L[1]*np.cos(theta[1])]]) * omega[1]
		+ np.array([[-1*L[0]*np.sin(theta[0])],
					   [L[0]*np.cos(theta[0])]]) * omega[0])
	omega32 = np.linalg.solve(A,b)
	omega[3] = omega32[0]
	omega[2] = omega32[1]

	return omega

def calcSpeed(theta,omega):
	'''Returns x,y speeds of Joints O,A,B,C and Foot Connection of four bar given speed of point O'''
	jointSpeed = np.zeros((5,2))
	Ospeed = np.zeros((1,2))
	jointSpeed[0,:] = Ospeed
	jointSpeed[1,:] = Ospeed + np.array([-1*L[1] * np.sin(theta[1]), L[1] * np.cos(theta[1])])*omega[1] #A
	jointSpeed[2,:] = Ospeed + np.array([-1*L[0] * np.sin(theta[0]), L[0] * np.cos(theta[0])])*omega[0]+ np.array([-1*L[3] * np.sin(theta[3]), L[3] * np.cos(theta[3])])*omega[3] #B
	jointSpeed[3,:] = Ospeed + np.array([-1*L[0] * np.sin(theta[0]), L[0] * np.cos(theta[0])])*omega[0]#C
	jointSpeed[4,:] = Ospeed + (np.array([-1*L[1]*np.sin(theta[1]),  L[1]*np.cos(theta[1])		  ])*omega[1]
								  +  np.array([-1*L[4]*np.sin(theta[2] + np.pi), L[4]*np.cos(theta[2] + np.pi)])*omega[2]) #F
	return jointSpeed

L = [0.0615, 0.0218, 0.0748, 0.0468, 0.0624]
i = 0

plt.figure(num = 1, figsize = (3,2))
plt.axes(frameon=False)
plt.gca().axes.get_xaxis().set_visible(False)
plt.gca().axes.get_yaxis().set_visible(False)
footPts = np.empty((360,2))
footSpeed = np.empty((360,2))
for theta1 in np.linspace(0,2*np.pi,360):
	theta = calcAngles(theta1)
	omega = calcAngSpeeds(theta)
	O, A, B, C, F  = calcPos(theta)
	_, _, _, _, Fs = calcSpeed(theta,omega)
	footPts[i,:] = F
	footSpeed[i,:] = Fs
	
	if i == 180 or i == 180+36 or i == 180 + 72:
		legPts = np.array([O, A, F, B, C])
		plt.plot(legPts[:,0],legPts[:,1],'k',lw = 0.3, color = str((252. - i)/90.))
	i += 1

plt.plot(footPts[:,0],footPts[:,1],lw = 1.5)
r = 0.0218
theta = np.linspace(0,2*np.pi,360)
plt.plot(r*np.cos(theta),r*np.sin(theta),lw = .3)
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('leg.pdf', bbox_inches='tight')

plt.figure(num = 2, figsize = (3,2))
theta = np.linspace(0,2*np.pi,360)
plt.axes(frameon=False)
plt.gca().axes.set_xticks([])
plt.gca().axes.set_yticks([])
plt.xlabel('y-velocity')
plt.ylabel('y-position')
plt.plot(footSpeed[:,1],footPts[:,1],'r',lw = 1.5)
#plt.axis('equal')
plt.savefig('legspeed.pdf', bbox_inches='tight')

plt.figure(num = 3, figsize = (3,2))
theta = np.linspace(0,2*np.pi,360)
plt.axes(frameon=False)
plt.gca().axes.get_xaxis().set_visible(False)
plt.gca().axes.get_yaxis().set_visible(False)

r = .026
i =0
for theta1 in np.linspace(0,2*np.pi,360):
	if i == 180 or i == 180+36 or i == 180 + 72:
		x = r*np.cos(theta1)
		y = r*np.sin(theta1)			
		plt.plot([0,x],[0,y],'k',lw = 0.5, color = str((252. - i)/90.))
	i += 1

plt.plot(r*np.cos(theta),r*np.sin(theta),lw = 2)
plt.axis('equal')
plt.savefig('leg2.pdf', bbox_inches='tight')

plt.figure(num = 4, figsize = (3,2))
theta = np.linspace(0,2*np.pi,360)
plt.axes(frameon=False)
plt.xlabel('y-velocity')
plt.ylabel('y-position')
plt.gca().axes.set_xticks([])
plt.gca().axes.set_yticks([])

plt.plot(70*r*np.cos(theta),r*np.sin(theta),'r',lw = 1.5)
#plt.axis('equal')
plt.savefig('leg2speed.pdf', bbox_inches='tight')

