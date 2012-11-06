import numpy as np
import matplotlib.pyplot as plt
import yaml
import robot
import leg
import motor
import footground as foot
from subprocess import call

movie = True

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

initLegPos = np.array([inputs['robot']['initPos'], np.array([0., 0.]), np.array([0., 0.])])
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
Foot = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams,robotParams['mass'])
Leg = leg.Leg(initLegAngle,initLegPos,legParams,Foot)
Motor = motor.Motor(motorParams,worldParams)
Robot = robot.Robot(robotParams,worldParams,(Leg,),(Motor,))

#set up list<D-]>s to store data
robotForces = np.zeros((len(time),2))
robotTorque = np.zeros(len(time))
robotpos = np.zeros((len(time),2))
robotspeed = np.zeros((len(time),2))
robotaccel = np.zeros((len(time),2))
ftpos = np.zeros((len(time),2))
ftspeed = np.zeros((len(time),2))
ftaccel = np.zeros((len(time),2))
footForce = np.zeros((len(time),2))

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
i = 0
j = 0
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
	F1 = Robot.legs[0].Foot.pos[1,:]
	F2 = Robot.legs[0].Foot.pos[2,:]
	ftspeed[i,:] = Robot.legs[0].Foot.speed
	ftaccel[i,:] = Robot.legs[0].Foot.accel
	legPts = np.array([O, A, F, F1, F2, F1, F, B, C, O])

	footForce[i,:] = [Robot.legs[0].Foot.loadx, Robot.legs[0].Foot.loady]
	
	if movie == True:
		if i ==0:
			plt.figure(figsize = (480./80.,320./80.), dpi = 80, num = 1)
			lines, = plt.plot(legPts[:,0],legPts[:,1],'-ok',lw = 0.2, markersize = 4, markeredgewidth = 0)
			plt.plot([-0.08, 0.10],[0, 0],'g')
			plt.axis((O[0] - 0.10, O[0] + 0.10, O[1] -0.10, O[1]+ 0.10))
			plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
			print 'frame ',j,' saved',Robot.time,' s'
			j += 1
		if i%100 == 0:
			lines.set_xdata(legPts[:,0])
			lines.set_ydata(legPts[:,1])
			plt.draw()
			plt.axis((O[0] - 0.10, O[0] + 0.10, O[1] -0.10 , O[1]+ 0.10))
			#plt.axis((-0.08, 0.10, -0.02, 0.12))
			plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
			print 'frame ',j,' saved',Robot.time,' s'
			j += 1
	i += 1

print 'force = ',robotForces
print 'pos = ',robotpos
print 'speed = ',robotspeed
print 'accel = ',robotaccel

print footForce.T
#plot results
if movie == True:
	call(['ffmpeg', '-i', './movie/leg%d.png', '-r', '30','-b', '400000', '-y', './movie/water.avi'])

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
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed'], loc = 6, bbox_to_anchor = (1.05, 0.5))

p3 = ax3.plot(time,robotaccel)
ax3.set_ylabel(r'Robot Accel $(\frac{m}{s^2})$')
ax3.set_xlabel('time (s)')
lgd3 = ax3.legend(p3, ['x-accel', 'y-accel'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('./plots/robot.png', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

#figure 3 forces
fig = plt.figure(num = 3)
tit = fig.suptitle('Joint Forces')
ax1 = fig.add_subplot(111)
ax2 = ax1.twinx()
p1, = ax1.plot(time,robotForces[:,0],label = 'Frx')
p2, = ax1.plot(time,robotForces[:,1],label = 'Fry')
p3, = ax2.plot(time,robotTorque,'r',label = 'Torque')
ax1.set_ylabel('Force (N)')
ax2.set_ylabel('Torque (N-m)')
ax1.set_xlabel('Time (s)')
x1, x2, y1,y2 = ax1.axis()
ax1.axis((x1,x2,y1-0.2,y2+0.2))
lines = [p1, p2, p3]
lgd = ax1.legend(lines, [l.get_label() for l in lines], loc = 6, bbox_to_anchor = (1.1,0.5))
plt.savefig('./plots/Force.png', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

#figure 4 foot position speed accel vs time
fig = plt.figure(num = 4)
tit = fig.suptitle('Foot Kinematics')
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)

p1 = ax1.plot(time,ftpos)
ax1.set_ylabel(r'Foot Coordinate (m)')
lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

p2 = ax2.plot(time,ftspeed)
ax2.set_ylabel(r'Foot Speed $(\frac{m}{s})$')
lgd2 = ax2.legend(p2, ['x-speed', 'y-speed'], loc = 6, bbox_to_anchor = (1.05, 0.5))

p3 = ax3.plot(time,ftaccel)
ax3.set_ylabel(r'Foot Accel $(\frac{m}{s^2})$')
ax3.set_xlabel('time (s)')
lgd3 = ax3.legend(p3, ['x-accel', 'y-accel'], loc = 6, bbox_to_anchor = (1.05, 0.5))

plt.savefig('./plots/foot.png', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

#figure 5 forces
fig = plt.figure(num = 5)
tit = fig.suptitle('Ground Reaction Forces')
ax1 = fig.add_subplot(111)
p1, = ax1.plot(time,footForce[:,0],label = 'Grx')
p2, = ax1.plot(time,footForce[:,1],label = 'Gry	')
ax1.set_ylabel('Force (N)')
ax1.set_xlabel('Time (s)')
x1, x2, y1,y2 = ax1.axis()
ax1.axis((x1,x2,y1-0.2,y2+0.2))
lines = [p1, p2]
lgd = ax1.legend(loc = 6, bbox_to_anchor = (1.1,0.5))
plt.savefig('./plots/GroundForce.png', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

