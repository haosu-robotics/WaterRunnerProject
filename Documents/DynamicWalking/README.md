Dynamic Walking 2013 Poster
===========================

This folder contains the poster for the [Dynamic Walking 2013][1] Conference. The controller presented in this conference is an iteration and simplification of the version presented in the KDC final project. It includes the following major changes:

	1. The system model used by the inverse dynamics controller is linearized about the desired
	   position and estimated control input at the desired position. In contrast the previous 
	   version of the controller linearized the model at every time step about the current 
	   position and control input.

	2. The controller only attempts to actively stabilize the height and roll of the robot. The
	   pitch is passively controlled via the tail.

	3. The leg velocities projected into the nullspace of the dynamics matrix are now obtained
	   via a heuristic that attempts to push the legs back to the desired phase relationship. 
	   This acts as a second CPG that respects the controller action.

	4. The disturbance wave was changed to a single pulse change in water level. This disturbance
	   is easier to conceptualize than the torque waves previously used. Also, this disturbance 
	   is not zero-mean, and thus poses a more difficult challenge than the continuous sine wave 
	   disturbance used in the KDC project.

[1]: http://www.cmu.edu/dynamic-walking/ "Dynamic Walking 2013"
