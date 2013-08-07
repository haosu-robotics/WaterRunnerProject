Water-Runner-Simulation-V2/Water
================================

Models
------

There are three models in this folder:

1. WaterRunner\_closed.slx - This model includes the full water runner model with a feedback linearizing controller used for the [KDC project][1]. This controller uses a system model linearized about the current position in order to control the height, roll, and pitch of the robot.

2. WaterRunner\_closed2.slx - This model includes the full water runner model and a controller that controls the height and roll of the robot by assuming the left and the right hand sides of the robot are decoupled. It then uses a PID controller to control the height of the left and the right hand sides of the robot. The pitch is controlled passively via the tail.

3. WaterRunner\_open.mdl - This model includes the full water runner model with no controller on height or roll or CPG maintaining phase differences. All legs driven at specified frequency.

4. WaterRunner.mdl - This model includes the full water runner model with no controller on height or roll. There is a CPG that maintains phase differences between legs.

simParam.m
----------

This file contains setup parameters and options and should be run before the simulation is started. There are several sections of parameters in this file:

1. Initial and desired conditions and disturbance inputs.
2. Robot Parameters
    1. Leg link lengths and frame size
    2. Robot mass is calculated from geometry and density of legs and assumed mass of the body.
3. Water Model Parameters
    1. Foot area
    2. Foot area folding ratio
    3. Drag coefficient
    4. Gravity
4. Controller Parameters
    1. PID gains tuned via Ziegler Nichols Method
    2. CPG parameters including phase lags between legs and CPG gain

heightSearch.m
--------------

Runs the WaterRunner.mdl model while sweeping leg frequency and then plots and saves in a .mat file the average steady state heights.

Linearized/
-----------

This folder contains Simulink models with the controller [presented at the dynamic walking conference][2]. This controller only attempts to control height and roll, and uses a model linearized about the desired position.

[1]: ../../Documents/KDCproject/FinalReport/final.pdf      "KDC project final report"
[2]: ../../Documents/DynamicWalking/poster.pdf             "Dynamic Walking 2013 poster"
