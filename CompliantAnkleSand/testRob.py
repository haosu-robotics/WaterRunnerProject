import numpy as np
import matplotlib.pyplot as plt
import yaml
import robot
import leg
import motor
import foot


#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

initLegPos = np.array([inputs['robot']['initPos'], [0., 0.], [0., 0.]])
initLegAngle = np.array([0., 0., 0.])
robotParams = inputs['robot']
legParams = inputs['leg']
footParams	= inputs['foot']
worldParams = inputs['world']
motorParams = inputs['motor']

timestep = worldParams['timeStep']
endtime = worldParams['endTime']
time = np.arange(0,endtime+timestep,timestep)

#initialize foot, leg, and robot
Foot = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams)
Leg = leg.Leg(initLegAngle,initLegPos,legParams,Foot)
Motor = motor.Motor(motorParams,worldParams)
Robot = robot.Robot(robotParams,worldParams,(Leg,),(Motor,))

#set up lists to store data
robotForces = np.zeros((len(time),2))
robotTorque = np.zeros(len(time))
robotpos = np.zeros((len(time),2))
robotspeed = np.zeros((len(time),2))
robotaccel = np.zeros((len(time),2))
ftpos = np.zeros((len(time),2))
ftspeed = np.zeros((len(time),2))
ftaccel = np.zeros((len(time),2))

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
i = 0
while Robot.time < endtime:
	#update robot
	Robot.update()
	
	#gather data
	jointPts = Robot.legs[0].jointPos
	O = jointPts[0,:]
	A = jointPts[1,:]
	B = jointPts[2,:]
	C = jointPts[3,:]
	F = jointPts[4,:]
	robotForces[i,:] = Robot.Force
	robotTorque[i] = Robot.motors[0].load
	robotpos[i,:] = Robot.pos
	robotspeed[i,:] = Robot.speed
	robotaccel[i,:] = Robot.accel
	ftpos[i,:] = Robot.legs[0].Foot.pos
	ftspeed[i,:] = Robot.legs[0].Foot.speed
	ftaccel[i,:] = Robot.legs[0].Foot.accel

	legPts = np.array([O, A, F, B, C])
	plt.plot(legPts[:,0],legPts[:,1],'k',lw = 0.2, markersize = 4, markeredgewidth = 0)
	i += 1

print 'force = ',robotForces
print 'pos = ',robotpos
print 'speed = ',robotspeed
print 'accel = ',robotaccel

#plot results

#figure 1 leg kinematics
plt.title('Leg Position Kinematics')
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('./plots2/leg.pdf', bbox_inches='tight')

#figure 2 robot position
fig = plt.figure(num = 2)
tit = fig.suptitle('Robot Position')
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)

p1 = ax1.plot(time,robotpos)
ax1.set_ylabel(r'Robot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,robotspeed)
ax2.set_ylabel(r'Robot Speed $(\frac{m}{s})$')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

p3 = ax3.plot(time,robotaccel)
ax3.set_ylabel(r'Robot Accel $(\frac{m}{s^2})$')
ax3.set_xlabel('time (s)')
lgd3 = ax3.legend(p3, ['x-accel', 'y-accel', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('./plots2/robot.pdf', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

#figure 3 forces
fig = plt.figure(num = 3)
tit = fig.suptitle('Joint Forces')
ax = fig.add_subplot(111)
p1 = ax.plot(time,robotForces[:,0],label = 'Frx')
p2 = ax.plot(time,robotForces[:,1],label = 'Fry')
p3 = ax.plot(time,robotTorque,label = 'torque')
ax.set_ylabel('Force (N)')
ax.set_xlabel('Time (s)')
x1, x2, y1,y2 = plt.axis()
plt.axis((x1,x2,y1-0.2,y2+0.2))
lgd = ax.legend(loc = 6, bbox_to_anchor = (1.05,0.5))
plt.savefig('./plots2/Force.pdf', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

#figure 4 foot position speed accel vs time
fig = plt.figure(num = 4)
tit = fig.suptitle('Foot Kinematics')
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)

p1 = ax1.plot(time,ftpos)
ax1.set_ylabel(r'Foot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,ftspeed,time,np.sum(ftspeed**2,axis=-1)**(1./2))
ax2.set_ylabel(r'Foot Speed $(\frac{m}{s})$')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

p3 = ax3.plot(time,ftaccel,time,np.sum(ftaccel**2,axis=-1)**(1./2))
ax3.set_ylabel(r'Foot Accel $(\frac{m}{s^2})$')
ax3.set_xlabel('time (s)')
lgd3 = ax3.legend(p3, ['x-accel', 'y-accel', 'magnitude'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('./plots2/foot.pdf', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')
