#! /usr/bin/env ipython

import numpy as np
import matplotlib.pyplot as plt
import yaml
import robot
import leg
import motor
import footgroundtest as foot
import os
from sys import platform

movie = True
plots = True

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

initLegPos = np.array([inputs['robot']['initPos'], np.array([0., 0.]), np.array([0., 0.])])
initLegAngle = np.array([0., 0., 0.])
robotParams = inputs['robot']
legParams = inputs['leg']
footParams	= inputs['footGround']
worldParams = inputs['world']
motorParams = inputs['motor']

timeStep = worldParams['timeStep']
timeStepFine = worldParams['timeStepFine']
frameRate = worldParams['frameRate']
curTimeStep = timeStep

endtime = worldParams['endTime']
time = [0]

#initialize foot, leg, and robot
Foot = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams,worldParams,robotParams['mass'])
Leg = leg.Leg(initLegAngle,initLegPos,legParams,worldParams,Foot)
Motor = motor.Motor(motorParams, 0)
Robot = robot.Robot(robotParams,worldParams,(Leg,),(Motor,))

#set up lists to store data
robotForces = []
robotTorque = []
robotpos = []
robotspeed = []
robotaccel = []
ftpos = []
ftspeed = []
ftaccel = []
footForce = []
footAngle = []

#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
i = 0
j = 0
firstFrame = True

while Robot.time < endtime:
	try:	
		#update robot
		Robot.update(curTimeStep)
		time.append(time[len(time)-1] + curTimeStep)
		
		#gather data
		jointPts = Robot.legs[0].jointPos
		O = jointPts[0,:]
		A = jointPts[1,:]
		B = jointPts[2,:]
		C = jointPts[3,:]
		F0 = jointPts[4,:]
		F1 = Robot.legs[0].Foot.pos[1,:]
		F2 = Robot.legs[0].Foot.pos[2,:]
		legPts = np.array([O, A, F0, F1, F2, F1, F0, B, C, O])

		robotForces.append(Robot.Force)
		robotTorque.append(Robot.motors[0].load)
		robotpos.append(Robot.pos)
		robotspeed.append(Robot.speed)
		robotaccel.append(Robot.accel)
		ftpos.append(Robot.legs[0].Foot.pos[0,:])
		ftspeed.append(Robot.legs[0].Foot.speed[0,:])
		ftaccel.append(Robot.legs[0].Foot.accel)
		footForce.append([Robot.legs[0].Foot.loadx, Robot.legs[0].Foot.loady])
		footAngle.append(Robot.legs[0].Foot.theta)
		
		if F2[1] < 0.001:
			curTimeStep = timeStepFine
		else:
			curTimeStep = timeStep

		if movie == True:
			if firstFrame:
				os.system('rm movie/*')
				fig = plt.figure(figsize = (480./80.,320./80.), dpi = 80, num = 1)
				ax1 = fig.add_subplot(111)
				lines, = ax1.plot(legPts[:,0],legPts[:,1],'-ok',lw = 0.2, markersize = 4, markeredgewidth = 0)
				ax1.plot([-100, 100],[0, 0],'b')
				ax1.axis((O[0] - 0.15, O[0] + 0.15, O[1] -0.15, O[1]+ 0.15))
				ax1.xaxis.grid(color='gray', linestyle='dashed')
				plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
				print 'frame ',j,' saved',Robot.time,' s'
				lastFrame = Robot.time
				firstFrame = False
				j += 1
			if Robot.time - lastFrame >= frameRate and not firstFrame:
				lines.set_xdata(legPts[:,0])
				lines.set_ydata(legPts[:,1])
				plt.draw()
				ax1.axis((O[0] - 0.15, O[0] + 0.15, O[1] -0.15 , O[1]+ 0.15))
				ax1.xaxis.grid(color='gray', linestyle='dashed')
				plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
				print 'frame ',j,' saved',Robot.time,' s'
				lastFrame = Robot.time
				j += 1
		i += 1
	except KeyboardInterrupt:
		break

#plot results
if movie == True:
	if platform == 'darwin':	
		os.system('ffmpeg -i ./movie/leg%d.png -s 700x522 -r 30 -qscale 1 -y ./movie/mov.mp4')
	else:
		os.system('avconv -i ./movie/leg%d.png -s 700x522 -r 30 -qscale 1 -y ./movie/mov.mp4')

if plots == True:
	time.pop()
	time = np.array(time)
	plotTimeStart = worldParams['plotTimeStart']
	plotTimeEnd = worldParams['plotTimeEnd']
	t1 = time >= plotTimeStart
	t2 = time <= plotTimeEnd
	timeToPlot = t1*t2

	robotForces = np.array(robotForces)
	robotTorque = np.array(robotTorque)
	robotpos = np.array(robotpos)
	robotspeed = np.array(robotspeed)
	robotaccel = np.array(robotaccel)
	ftpos = np.array(ftpos)
	ftspeed = np.array(ftspeed)
	ftaccel = np.array(ftaccel)
	footForce = np.array(footForce)
	footAngle = np.array(footAngle)
	
	#figure 2 robot position
	fig = plt.figure(num = 2)
	tit = fig.suptitle('Robot Position')
	ax1 = fig.add_subplot(311)
	ax2 = fig.add_subplot(312)
	ax3 = fig.add_subplot(313)
	
	p1 = ax1.plot(time[timeToPlot],robotpos[timeToPlot])
	ax1.set_ylabel(r'Robot Coordinate (m)')
	lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

	p2 = ax2.plot(time[timeToPlot],robotspeed[timeToPlot])
	ax2.set_ylabel(r'Robot Speed $(\frac{m}{s})$')
	lgd2 = ax2.legend(p2, ['x-speed', 'y-speed'], loc = 6, bbox_to_anchor = (1.05, 0.5))

	p3 = ax3.plot(time[timeToPlot],robotaccel[timeToPlot])
	ax3.set_ylabel(r'Robot Accel $(\frac{m}{s^2})$')
	ax3.set_xlabel('time (s)')
	lgd3 = ax3.legend(p3, ['x-accel', 'y-accel'], loc = 6, bbox_to_anchor = (1.05, 0.5))

	plt.savefig('./plots/robot.png', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

	#figure 3 forces
	fig = plt.figure(num = 3,figsize = (8,6))
	#tit = fig.suptitle('Joint Forces')
	ax1 = fig.add_subplot(111)
	ax2 = ax1.twinx()
	p1, = ax1.plot(time[timeToPlot],robotForces[timeToPlot,1],label = r'$F_y$')
	p2, = ax2.plot(time[timeToPlot],robotTorque[timeToPlot],'r',linewidth = 0.5, label = 'Torque')
	ax1.set_ylabel('Force (N)')
	ax2.set_ylabel('Torque (N-m)')
	ax1.set_xlabel('Time (s)')
	x1, x2, y1, y2 = ax1.axis()
	#ax1.axis((x1,0.75,-1.5,1.5))
	#ax2.axis((x1,0.75,0,100))
	lines = [p1, p2]
	lgd = ax1.legend(lines, [l.get_label() for l in lines], loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/Force.pdf', bbox_extra_artists = (lgd,), bbox_inches = 'tight')

	#figure 4 foot position speed accel vs time
	fig = plt.figure(num = 4)
	tit = fig.suptitle('Foot Kinematics')
	ax1 = fig.add_subplot(311)
	ax2 = fig.add_subplot(312)
	ax3 = fig.add_subplot(313)

	p1 = ax1.plot(time[timeToPlot],ftpos[timeToPlot])
	ax1.set_ylabel(r'Foot Coordinate (m)')
	lgd1 = ax1.legend(p1, ['x-position', 'y-position'], loc = 6, bbox_to_anchor = (1.05,0.5))

	p2 = ax2.plot(time[timeToPlot],ftspeed[timeToPlot])
	ax2.set_ylabel(r'Foot Speed $(\frac{m}{s})$')
	lgd2 = ax2.legend(p2, ['x-speed', 'y-speed'], loc = 6, bbox_to_anchor = (1.05, 0.5))

	p3 = ax3.plot(time[timeToPlot],ftaccel[timeToPlot])
	ax3.set_ylabel(r'Foot Accel $(\frac{m}{s^2})$')
	ax3.set_xlabel('time (s)')
	lgd3 = ax3.legend(p3, ['x-accel', 'y-accel'], loc = 6, bbox_to_anchor = (1.05, 0.5))

	plt.savefig('./plots/foot.png', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

	#figure 5 forces
	fig = plt.figure(num = 5)
	tit = fig.suptitle('Ground Reaction Forces')
	ax1 = fig.add_subplot(111)
	p1, = ax1.plot(time[timeToPlot],footForce[timeToPlot,0],label = 'Grx')
	p2, = ax1.plot(time[timeToPlot],footForce[timeToPlot,1],label = 'Gry	')
	ax1.set_ylabel('Force (N)')
	ax1.set_xlabel('Time (s)')
	x1, x2, y1,y2 = ax1.axis()
	#ax1.axis((x1,x2,y1-0.2,y2+0.2))
	lines = [p1, p2]
	lgd = ax1.legend(loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/GroundForce.png', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

