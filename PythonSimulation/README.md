Python Planar Water Runner Simulation
=====================================

This folder contains four python simulations of planer water runner robots:

	1. testRob.py
		* Single leg
		* Hard ground

	2. testRobWater.py
		* Single leg
		* Water

	3. testTwoLeg.py
		* Two legs
		* Hard ground
		
	4. testTwoLegWater.py
		* Two legs
		* Water

The inputs.yaml file contains all settings for the simulations including robot and motor geometric and inertial properties, world properties such as gravity, and initial conditions. For water running and ground running respectively the following ballpark settings should be used:
	
	* For water running:
		* robot.initPos = [0, 0.25]
		* motor.initSpeed = 70.

	* For ground running:
		* robot.initPos = [0, 0.70]
		* motor.initSpeed = 20.

The simulations can be ran on a computer with python and the required packages by running ./*name of simulation*.py. The simulations will run for the amount of time specified in the inputs.yaml file or until the user presses Ctrl-C. When the simulation ends via either of these two methods, an animation of the movie is saved in the movie folder and data is plotted and saved in the plots folder.

For more information regarding the calculations performed by this simulation see [this document](../Documents/SimWriteup/report.pdf)
