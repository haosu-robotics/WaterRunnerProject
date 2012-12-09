clc,
close all
clear all;

simParams;

load_system('WaterRunner.mdl')
sim('WaterRunner',10);
close_system('WaterRunner.mdl')

