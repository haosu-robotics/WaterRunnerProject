import numpy as np
import matplotlib.pyplot as plt
import yaml
import robot
import leg
import motor
import footgroundtest as foot
import os
from sys import platform
from copy import copy
import pdb

movie = True
plots = True

#load set up file
inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)
leg0pos = np.array(inputs['robot']['leg0Pos'])
initLegPos0 = np.array([leg0pos, [0., 0.],[0., 0.]])
initLegAngle0 = np.array([0., 0., 0.])

leg1pos = np.array(inputs['robot']['leg1Pos'])
initLegPos1 = np.array([leg1pos, [0., 0.], [0., 0.]])
initLegAngle1 = np.array([np.pi, 0., 0.])

robotParams = inputs['robot']
legParams = inputs['leg']
footParams	= inputs['footGround']
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
Foot1 = foot.Foot(np.zeros(3),np.zeros((3,2)),footParams,worldParams,robotParams['mass'])
Leg0 = leg.Leg(initLegAngle0,initLegPos0,legParams,worldParams,Foot0)
Leg1 = leg.Leg(initLegAngle1,initLegPos1, legParams, worldParams, Foot1)
Motor0 = motor.Motor(motorParams, 0.)
Motor1 = motor.Motor(motorParams, np.pi)
Robot = robot.Robot(robotParams,worldParams,(Leg0, Leg1),(Motor0, Motor1))

#set up lists to store data
robotForces = [[], []]
robotTorque = [[], []]
robotpos = [[], []]
robotspeed = [[], []]
robotaccel = [[], []]
ftpos = [[], []]
ftspeed = [[], []]
ftaccel = [[], []]
footForce = [[], []]
footAngle = [[], []]
legPts = [np.inf, np.inf]
#calculate positions, speeds, and accels vs angle
plt.figure(num = 1)
j = 0
firstFrame = True

while Robot.time < endtime:
	try:	
		#update robot
		Robot.update(curTimeStep)
		time.append(time[len(time)-1] + curTimeStep)
		
		F2 =[np.inf*np.ones(2),np.inf*np.ones(2)]
		#gather data
		for num, leg in enumerate(Robot.legs):
			jointPts = leg.jointPos
			O = jointPts[0,:]
			A = jointPts[1,:]
			B = jointPts[2,:]
			C = jointPts[3,:]
			F0 = jointPts[4,:]
			F1 = leg.Foot.pos[1,:]
			F2[num] = leg.Foot.pos[2,:]
			legPts[num] = np.array([O, A, F0, F1, F2[num], F1, F0, B, C, O])
			
			robotForces[num].append(copy(Robot.Force))
			robotTorque[num].append(copy(Robot.motors[num].load))
			robotpos[num].append(copy(Robot.pos))
			robotspeed[num].append(copy(Robot.speed))
			robotaccel[num].append(copy(Robot.accel))
			ftpos[num].append(copy(leg.Foot.pos[0,:]))
			ftspeed[num].append(copy(leg.Foot.speed[0,:]))
			ftaccel[num].append(copy(leg.Foot.accel))
			footForce[num].append([copy(leg.Foot.loadx), copy(leg.Foot.loady)])
			footAngle[num].append(copy(leg.Foot.theta))
			
			#pdb.set_trace()
			if F2[0][1] < 0.001 or F2[1][1] < 0.001:
				curTimeStep = timeStepFine
			else:
				curTimeStep = timeStep
			
		if movie == True:
			if firstFrame:
				os.system('rm movie/*')
				fig = plt.figure(figsize = (480./80.,320./80.), dpi = 80, num = 1)
				ax1 = fig.add_subplot(111)
				lines0, = ax1.plot(legPts[0][:,0],legPts[0][:,1],'-ok',lw = 0.2, markersize = 4, markeredgewidth = 0)
				lines1, = ax1.plot(legPts[1][:,0],legPts[1][:,1],'-ok',lw = 0.2, markersize = 4, markeredgewidth = 0)
				lines2, = ax1.plot([legPts[0][0,0], legPts[1][0,0]],[legPts[0][0,1], legPts[1][0,1]],'k', lw = 0.3, markeredgewidth = 0)
				ax1.plot([-100, 100],[0, 0],'b')
				ax1.axis((O[0] - 0.30, O[0] + 0.15, -0.05, 0.30))
				ax1.xaxis.grid(color='gray', linestyle='dashed')
				plt.savefig(''.join(['./movie/leg', str(j), '.png']), bbox_inches='tight')
				print 'frame ',j,' saved',Robot.time,' s'
				lastFrame = Robot.time
				firstFrame = False
				j+= 1
			if Robot.time - lastFrame >= frameRate and not firstFrame:
				lines0.set_xdata(legPts[0][:,0])
				lines0.set_ydata(legPts[0][:,1])
				lines1.set_xdata(legPts[1][:,0])
				lines1.set_ydata(legPts[1][:,1])
				lines2.set_xdata([legPts[0][0,0], legPts[1][0,0]])
				lines2.set_ydata([legPts[0][0,1], legPts[1][0,1]])
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
		os.system('ffmpeg -i ./movie/leg%d.png -s 700x522 -r 30 -qscale 1 -y ./movie/aa.mp4')
	else:
		os.system('avconv -i ./movie/leg%d.png -s 700x522 -r 30 -qscale 1 -y ./movie/aa.mp4')

if plots == True:
	time.pop()
	time = np.array(time)
	robotForces = np.array(robotForces[1])
	robotTorque = np.array(robotTorque[1])
	robotpos = np.array(robotpos[1])
	robotspeed = np.array(robotspeed[1])
	robotaccel = np.array(robotaccel[1])
	ftpos = np.array(ftpos[1])
	ftspeed = np.array(ftspeed[1])
	ftaccel = np.array(ftaccel[1])
	footForce = np.array(footForce[1])
	footAngle = np.array(footAngle[1])
	
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
	fig = plt.figure(num = 3,figsize = (8,6))
	#tit = fig.suptitle('Joint Forces')
	ax1 = fig.add_subplot(111)
	#p1, = ax1.plot(time,robotForces[:,1],label = r'$F_y$')
	p2, = ax1.plot(time,robotTorque,'r',linewidth = 0.5, label = 'Torque')
	#ax1.set_ylabel('Force (N)')
	ax1.set_ylabel('Torque (N-m)')
	ax1.set_xlabel('Time (s)')
	x1, x2, y1, y2 = ax1.axis()
	#ax1.axis((x1,0.75,-1.5,1.5))
	#ax2.axis((x1,0.75,0,100))
	lines = [p2, ] #[p1, p2]
	lgd = ax1.legend(lines, [l.get_label() for l in lines], loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/Torque.pdf', bbox_extra_artists = (lgd,), bbox_inches = 'tight')

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
	#ax1.axis((x1,x2,y1-0.2,y2+0.2))
	lines = [p1, p2]
	lgd = ax1.legend(loc = 6, bbox_to_anchor = (1.1,0.5))
	plt.savefig('./plots/GroundForce.png', bbox_extra_artists = (lgd,tit), bbox_inches = 'tight')

#pdb.set_trace()
