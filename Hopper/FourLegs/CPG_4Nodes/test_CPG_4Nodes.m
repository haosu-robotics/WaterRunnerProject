clc
close all
clear all



freq = 10;
amp = 1;

                    % in our case, should satisfy l1-l2-l3+l4 = 0
                    % for trot: l2+l3 = 0 (not 2pi) and 11+l4 = 0 (not 2pi)
FR_FL_lag = pi;     % l1
HR_HL_lag = -pi;    % l2
FR_HR_lag = pi;     % l3
FL_HL_lag = -pi;    % l4


cw_FR_FL = 5;
cw_HR_HL = 5;
cw_FR_HR = 5;
cw_FL_HL = 5;

alpha = 30;
tau = 0.001;
