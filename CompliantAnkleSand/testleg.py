import numpy as np
import matplotlib.pyplot as plt
import yaml
from leg import *

inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

linkLengths = inputs['linkLengths']
leg1 = Leg(linkLengths,np.array([0, 0]),1)

plt.figure()
i = 0
footPts = np.empty((360,2))
angle = np.empty((360,4))
speed = np.empty((360,4))
accel = np.empty((360,4))

omega1 = 2*np.pi
alpha1 = 0
plt.figure(num = 1)

for theta1 in np.linspace(0,2*np.pi,360):
	angle[i,:] = (180./(np.pi))*leg1.calcAngles(theta1)
	speed[i,:] = (1./(2*np.pi))*leg1.calcAngSpeeds(omega1)
	accel[i,:] = (1./(2*np.pi))*leg1.calcAngAccels(alpha1)

	O, A, B, C, F = leg1.calcPos()
	footPts[i,:] = F
	if np.mod(i,15) == 0:
		legPts = np.array([O, A, F, B, C])
		plt.plot(legPts[:,0],legPts[:,1],'k',lw = 0.2, color = str(i/(360.+45.)), markersize = 4, markeredgewidth = 0)
	i += 1

print 'max angle = ',np.amax(angle, axis = 0)
print 'min angle = ',np.amin(angle, axis = 0)

print 'max speed = ',np.amax(speed, axis = 0)
print 'min speed = ',np.amin(speed, axis = 0)

print 'max accel = ',np.amax(accel, axis = 0)
print 'min accel = ',np.amin(accel, axis = 0)


plt.title('Leg Position Kinematics')
plt.plot(footPts[:,0],footPts[:,1])
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('leg.pdf', bbox_inches='tight')


fig = plt.figure(num = 2)
tit = fig.suptitle('Leg Angular Kinematics')
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)
time = np.linspace(0,1,360) 
ax1.plot(time,angle)
ax1.set_ylabel('Angle (Degrees)')
ax2.plot(time,speed)
ax2.set_ylabel(r'Speed $\frac{rev}{sec}$')
p3 = ax3.plot(time,accel)
ax3.set_ylabel(r'Acceleration $\frac{rev}{sec^2}$')
ax3.set_xlabel('time (s)')
lgd = ax2.legend(p3, ['Joint 0', 'Joint 1', 'Joint 2', 'Joint 3'], loc = 6, bbox_to_anchor = (1.05, 0.5))
plt.savefig('angles.pdf', bbox_extra_artists=(lgd,tit),  bbox_inches = 'tight')

