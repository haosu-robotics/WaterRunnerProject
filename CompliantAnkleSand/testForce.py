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
jointForces = np.empty((360,9))

omega1 = 2*np.pi
alpha1 = 0

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
for theta1 in np.linspace(0,2*np.pi,360):
	angle[i,:] = (180./(np.pi))*Leg1.calcAngles(theta1)
	speed[i,:] = (1./(2*np.pi))*Leg1.calcAngSpeeds(omega1)
	accel[i,:] = (1./(2*np.pi))*Leg1.calcAngAccels(alpha1)
	ftspeed[i,:] = Leg1.Foot.calcSpeed().T
	jointForces[i,:] = Leg1.calcForceTorque()
	print jointForces[i,:]
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
#plt.savefig('leg.pdf', bbox_inches='tight')

fig = plt.figure(num = 2, figsize = (10,10))
tit = fig.suptitle('Leg Angular Kinematics')
ax1 = fig.add_subplot(411)
ax2 = fig.add_subplot(412)
ax3 = fig.add_subplot(413)
time = np.linspace(0,1,360) 
ax1.plot(time,angle)
ax1.set_ylabel('Angle (Degrees)')

p2 = ax2.plot(time,speed)
ax2.set_ylabel(r'Speed $\frac{rev}{sec}$')

ax3.plot(time,accel)
ax3.set_ylabel(r'Acceleration $\frac{rev}{sec^2}$')

lgd= ax2.legend(p2, ['Joint 0', 'Joint 1', 'Joint 2', 'Joint 3'], loc = 6, bbox_to_anchor = (1.05, 0.5))

#plt.savefig('angles.pdf', bbox_extra_artists=(lgd,tit),  bbox_inches = 'tight')

fig3 = plt.figure(num = 3)
tit = fig3.suptitle('Foot Kinematics')
ax1 = fig3.add_subplot(211)
ax2 = fig3.add_subplot(212)

p1 = ax1.plot(time,footPts)
ax1.set_ylabel('Foot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,ftspeed,time,np.sum(ftspeed**2,axis=-1)**(1./2))
ax2.set_ylabel(r'Foot Speed $\frac{m}{s}$')
ax2.set_xlabel('time (s)')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('foot.pdf', bbox_extra_artists=(lgd1,lgd2,tit),  bbox_inches = 'tight')

fig4 = plt.figure(num = 4)
tit = fig4.suptitle('Joint Forces')
ax1 = fig4.add_subplot(111)

p1 = ax1.plot(time,jointForces[:,0:8])
ax1.set_ylabel('Force (N)')
ax1.set_xlabel('Time (s)')

lgd = ax1.legend(p1,['Fr1x', 'Fr1y', 'Fr2x', 'Fr2y', 'F2x', 'F2y', 'F4x', 'F4y'], loc = 6, bbox_to_anchor = (1.05,0.5))

#plt.savefig('Force.pdf', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

# force square figure and square axes looks better for polar, IMO
# make a square figure
fig = plt.figure(figsize=(8, 8), num = 5)
ax = fig.add_subplot(111, polar=True)
tit = fig.suptitle('Input Torque')

ax.plot(time*2*np.pi,jointForces[:,8]+ 0.12)
ax.set_rmax(0.04+0.12)
rng = np.arange(0.04,0.08+0.12,0.04)
ax.set_rgrids(rng, labels = ['-0.08', '-0.04','0.00'], angle = 0, label = 'Torque (N-m)')

#plt.savefig('TorquePolar.pdf', bbox_extra_artists = (tit,), bbox_inches = 'tight')

fig = plt.figure(figsize=(8, 8), num = 6)
ax = fig.add_subplot(111)
tit = fig.suptitle('Input Torque')

ax.plot(time,jointForces[:,8])
ax.set_ylabel('Torque (N-m)')
ax.set_xlabel('Time (s)')

#plt.savefig('Torque.pdf', bbox_extra_artists = (tit,), bbox_inches = 'tight')
