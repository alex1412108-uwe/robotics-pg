COM_CloseNXT all;

COM_CloseNXT all
close all
clear all
format compact

Ports = [MOTOR_A; MOTOR_B; MOTOR_C; SENSOR_1];

h=COM_OpenNXT(); 
COM_SetDefaultNXT(h); % begining bit 

OpenUltrasonic(SENSOR_1); 

r = realRobot;

map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
target = [30,30]; 

%%%%%%% bit for the localise

%  localise(r, map, target);  

%  h = figure;
%  hold on; 

%%%%%% bit for testing ultrascan
% sense = r.ultraScan 

%%%%%% bit for testing move
% move = 5;
% r.move(move); 

%%%%%% bit for testing turn
 turn = pi/2;
 r.turn(turn);

CloseSensor(SENSOR_4); 
COM_CloseNXT(h);
clear all; 