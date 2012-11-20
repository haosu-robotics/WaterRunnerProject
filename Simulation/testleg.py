import numpy as np
import matplotlib.pyplot as plt
import yaml
import leg
import foot

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

linkLengths = inputs['linkLengths']
footParams	= inputs['footpad']

#initialize Leg and foot
Leg1 = leg.Leg(linkLengths,np.array([0, 0]),1,footParams)

#set up vectors to store data
i = 0
footPts = np.empty((360,2))
angle = np.empty((360,4))
speed = np.empty((360,4))
accel = np.empty((360,4))
ftspeed = np.empty((360,2))

omega1 = 2*np.pi
alpha1 = 0

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
for theta1 in np.linspace(0,2*np.pi,360):
	angle[i,:] = (180./(np.pi))*Leg1.calcAngles(theta1)
	speed[i,:] = (1./(2*np.pi))*Leg1.calcAngSpeeds(omega1)
	accel[i,:] = (1./(2*np.pi))*Leg1.calcAngAccels(alpha1)
	ftspeed[i,:] = Leg1.Foot.calcSpeed().T

	O, A, B, C, F = Leg1.calcPos()
	footPts[i,:] = F
	if np.mod(i,15) == 0:
		legPts = np.array([O, A, F, B, C])
		plt.plot(legPts[:,0],legPts[:,1],'k',lw = 0.2, color = str(i/(360.+45.)), markersize = 4, markeredgewidth = 0)
	i += 1

#print maxes and mins for the calculated data
print 'max angle = ',np.amax(angle, axis = 0)
print 'min angle = ',np.amin(angle, axis = 0)

print 'max speed = ',np.amax(speed, axis = 0)
print 'min speed = ',np.amin(speed, axis = 0)

print 'max accel = ',np.amax(accel, axis = 0)
print 'min accel = ',np.amin(accel, axis = 0)

#plot results
plt.title('Leg Position Kinematics')
plt.plot(footPts[:,0],footPts[:,1])
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('leg.pdf', bbox_inches='tight')

fig = plt.figure(num = 2, figsize = (10,10))
tit = fig.suptitle('Leg Angular Kinematics')
ax1 = fig.add_subplot(411)
ax2 = fig.add_subplot(412)
ax3 = fig.add_subplot(413)
#fig.subplots_adjust(hspace = 1)
time = np.linspace(0,1,360) 
ax1.plot(time,angle)
ax1.set_ylabel('Angle (Degrees)')

p2 = ax2.plot(time,speed)
ax2.set_ylabel(r'Speed $\frac{rev}{sec}$')

ax3.plot(time,accel)
ax3.set_ylabel(r'Acceleration $\frac{rev}{sec^2}$')

lgd= ax2.legend(p2, ['Joint 0', 'Joint 1', 'Joint 2', 'Joint 3'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('angles.pdf', bbox_extra_artists=(lgd,tit),  bbox_inches = 'tight')

fig3 = plt.figure(num = 3)
tit = fig3.suptitle('Foot Kinematics')
ax1 = fig3.add_subplot(211)
ax2 = fig3.add_subplot(212)

p1 = ax1.plot(time,footPts)
ax1.set_ylabel('Foot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,ftspeed,time,np.sum(np.abs(ftspeed)**2,axis=-1)**(1./2))
ax2.set_ylabel(r'Foot Speed $\frac{m}{s}$')
ax2.set_xlabel('time (s)')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('foot.pdf', bbox_extra_artists=(lgd1,lgd2,tit),  bbox_inches = 'tight')
