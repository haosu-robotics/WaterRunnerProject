#! /usr/bin/env ipython

import numpy as np
import matplotlib.pyplot as plt
import yaml
import robot
import leg
import motor
import footwater as foot
import os
from sys import platform
from copy import copy

movie = True
plots = True

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)
leg0pos = np.array(inputs['robot']['leg0Pos'])
initLegPos0 = np.array([leg0pos, [0., 0.],[0., 0.]])
initLegAngle0 = np.array([0., 0., 0.])

robotParams = inputs['robot']
legParams = inputs['leg']
footParams	= inputs['footWater']
worldParams = inputs['world']
motorParams = inputs['motor']

timeStep = worldParams['timeStep']
timeStepFine = worldParams['timeStepFine']
curTimeStep = timeStep
frameRate = worldParams['frameRate']

endtime = worldParams['endTime']
time = [0]

#initialize foot, leg, and robot
Foot0 = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams,worldParams,robotParams['mass'])
Leg0 = leg.Leg(initLegAngle0,initLegPos0,legParams,worldParams,Foot0)
Motor0 = motor.Motor(motorParams, 0.)
Robot = robot.Robot(robotParams,worldParams,(Leg0,),(Motor0,))

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
legPts = []
#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
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
		F = jointPts[4,:]
		legPts = np.array([O, A, F, B, C, O])
		
		robotForces.append(copy(Robot.Force))
		robotTorque.append(copy(Robot.motors[0].load))
		robotpos.append(copy(Robot.pos))
		robotspeed.append(copy(Robot.speed))
		robotaccel.append(copy(Robot.accel))
		ftpos.append(copy(Robot.legs[0].Foot.pos))
		ftspeed.append(copy(Robot.legs[0].Foot.speed))
		ftaccel.append(copy(Robot.legs[0].Foot.accel))
		footForce.append([copy(Robot.legs[0].Foot.loadx), copy(Robot.legs[0].Foot.loady)])
		footAngle.append(copy(Robot.legs[0].Foot.theta))
			
		if movie == True:
			if firstFrame:
				os.system('rm movie/*')
				fig = plt.figure(figsize = (480./80.,320./80.), dpi = 80, num = 1)
				ax1 = fig.add_subplot(111)
				lines0, = ax1.plot(legPts[:,0],legPts[:,1],'-ok',lw = 0.2, markersize = 4, markeredgewidth = 0)
				ax1.plot([-100, 100],[0, 0],'b')
				ax1.axis((O[0] - 0.30, O[0] + 0.15, -0.05, 0.30))
				ax1.xaxis.grid(color='gray', linestyle='dashed')
				plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
				print 'frame ',j,' saved',Robot.time,' s'
				lastFrame = Robot.time
				firstFrame = False
				j+= 1
			if Robot.time - lastFrame >= frameRate and not firstFrame:
				lines0.set_xdata(legPts[:,0])
				lines0.set_ydata(legPts[:,1])
				plt.draw()
				ax1.axis((O[0] - 0.30, O[0] + 0.15, -0.05 , 0.30))
				ax1.xaxis.grid(color='gray', linestyle='dashed')
				plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
				print 'frame ',j,' saved',Robot.time,' s'
				lastFrame = Robot.time
				j += 1
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

	plt.savefig('./plots/robot.pdf', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')

	#figure 3 forces
	fig = plt.figure(num = 3,figsize = (8,6))
	tit = fig.suptitle('Joint Forces')
	ax1 = fig.add_subplot(111)
	ax2 = ax1.twinx()
	p1, = ax1.plot(time[timeToPlot],robotForces[timeToPlot,0],label = r'$F_x$')
	p2, = ax1.plot(time[timeToPlot],robotForces[timeToPlot,1],label = r'$F_y$')
	p3, = ax2.plot(time[timeToPlot],footAngle[timeToPlot],'r',linewidth = 0.5, label = 'Angle')
	ax1.set_ylabel('Force (N)')
	ax2.set_ylabel('Angle (radians)')
	ax1.set_xlabel('Time (s)')
	x1, x2, y1, y2 = ax1.axis()
	#ax1.axis((x1,0.75,-1.5,1.5))
	#ax2.axis((x1,0.75,0,100))
	lines = [p1, p2, p3]
	lgd = ax1.legend(lines, [l.get_label() for l in lines], loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/Force.pdf', bbox_extra_artists = (lgd,), bbox_inches = 'tight')

	#figure 4 forces
	fig = plt.figure(num = 5)
	tit = fig.suptitle('Ground Reaction Forces')
	ax1 = fig.add_subplot(111)
	ax2 = ax1.twinx()
	p1, = ax1.plot(time[timeToPlot],footForce[timeToPlot,0],label = 'Grx')
	p2, = ax1.plot(time[timeToPlot],footForce[timeToPlot,1],label = 'Gry')
	p3, = ax2.plot(time[timeToPlot],footAngle[timeToPlot],'r',linewidth = 0.5, label = 'Angle')
	ax1.set_ylabel('Force (N)')
	ax1.set_xlabel('Time (s)')
	ax2.set_ylabel('Angle (radians)')
	x1, x2, y1,y2 = ax1.axis()
	#ax1.axis((x1,x2,y1-0.2,y2+0.2))
	lines = [p1, p2, p3]
	lgd = ax1.legend(lines, [l.get_label() for l in lines], loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/GroundForce.pdf', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')
	
	#figure 5 foot position speed accel vs time
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

	plt.savefig('./plots/foot.pdf', bbox_extra_artists=(lgd1,lgd2,lgd3,tit),  bbox_inches = 'tight')
#pdb.set_trace()
