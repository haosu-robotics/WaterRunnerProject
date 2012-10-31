import numpy as np
import matplotlib.pyplot as plt
import yaml
import leg
import foot

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

initLegPos = np.array([inputs['robot']['initPos'], [0., 0.], [0., 0.]])
initLegAngle = np.array([0., 0., 0.])
legParams = inputs['leg']
footParams	= inputs['foot']

#initialize Leg and foot
Foot = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams)
Leg = leg.Leg(initLegAngle,initLegPos,legParams,Foot)

#set up vectors to store data
i = 0
angle = np.empty((360,4))
speed = np.empty((360,4))
accel = np.empty((360,4))
jointForces = np.empty((360,9))
footPts = np.empty((360,2))
ftspeed = np.empty((360,2))
ftaccel = np.empty((360,2))

omega1 = 2*np.pi
alpha1 = 0

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
for theta1 in np.linspace(0,2*np.pi,360):
	#update leg, which cascades and updates foot aswell
	Leg.update(np.array([theta1,omega1,0]), initLegPos)
	
	#gather data
	angle[i,:] = (180./(np.pi))*Leg.theta
	speed[i,:] = (1./(2*np.pi))*Leg.omega
	accel[i,:] = (1./(2*np.pi))*Leg.alpha
	jointPts = Leg.jointPos
	O = jointPts[0,:]
	A = jointPts[1,:]
	B = jointPts[2,:]
	C = jointPts[3,:]
	F = jointPts[4,:]
	jointForces[i,:] = Leg.jointLoad
	
	footPts[i,:] = Leg.Foot.pos
	ftspeed[i,:] = Leg.Foot.speed
	ftaccel[i,:] = Leg.Foot.accel

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
#figure 1 leg position
plt.title('Leg Position Kinematics')
plt.plot(footPts[:,0],footPts[:,1])
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('./plots/leg.pdf', bbox_inches='tight')

#figure 2 leg angles vs time
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

plt.savefig('./plots/angles.pdf', bbox_extra_artists=(lgd,tit),  bbox_inches = 'tight')

#figure 3 foot position speed accel vs time
fig3 = plt.figure(num = 3)
tit = fig3.suptitle('Foot Kinematics')
ax1 = fig3.add_subplot(311)
ax2 = fig3.add_subplot(312)
ax3 = fig3.add_subplot(313)

p1 = ax1.plot(time,footPts)
ax1.set_ylabel(r'Foot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,ftspeed,time,np.sum(ftspeed**2,axis=-1)**(1./2))
ax2.set_ylabel(r'Foot Speed $(\frac{m}{s})$')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

p3 = ax3.plot(time,ftaccel,time,np.sum(ftaccel**2,axis=-1)**(1./2))
ax3.set_ylabel(r'Foot Accel $(\frac{m}{s^2})$')
ax3.set_xlabel('time (s)')
lgd3 = ax3.legend(p3, ['x-accel', 'y-accel', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('./plots/foot.pdf', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

#figure 4 joint forces
fig4 = plt.figure(num = 4)
tit = fig4.suptitle('Joint Forces')
ax1 = fig4.add_subplot(111)

p1 = ax1.plot(time,jointForces[:,0:4])
ax1.set_ylabel('Force (N)')
ax1.set_xlabel('Time (s)')

lgd = ax1.legend(p1,['Fr1x', 'Fr1y', 'Fr2x', 'Fr2y'], loc = 6, bbox_to_anchor = (1.05,0.5))

plt.savefig('./plots/Force.pdf', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

#figure 5 torque on input shaft
fig = plt.figure(figsize=(8, 8), num = 5)
ax = fig.add_subplot(111)
tit = fig.suptitle('Input Torque')

ax.plot(time,jointForces[:,8])
ax.set_ylabel('Torque (N-m)')
ax.set_xlabel('Time (s)')

plt.savefig('./plots/Torque.pdf', bbox_extra_artists = (tit,), bbox_inches = 'tight')
