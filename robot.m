%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Robotic Systems March 2015 %%%%%%%%%%
%%%%%%%%%% Team LDCA - Lost Robot Cwk %%%%%%%%%%
%%%%%%%%%% Main fnc in real robot task %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Close any previously open NXT and prep 
COM_CloseNXT all;
close all
clear all
format compact

%%%% Define ports for use 
Ports = [MOTOR_A; MOTOR_B; MOTOR_C; SENSOR_1];

%%%% Open ports
h=COM_OpenNXT(); 
COM_SetDefaultNXT(h); 

%%%% Open sensor 
OpenUltrasonic(SENSOR_1); 

%%%% define robot object
r = realRobot;


%%%%%%%%%%%%%%%%%%%%%%%%%% To be defined on testing day

%%%% define map (currently set to the practise map)
map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];

%%%% define a target (currently set to arbritray point) 
target = [80, 80]; 


%%%%%% ^^^^^^ For testing only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic %starts timer

%%%% Create a figure for drawing 
figure;
hold on; 

%%%% Call the localise function using: robot, map and target
localise(r, map, target);  

resultsTime = toc %stops timer

%calculated how far away your robot thinks it is from the target
resultsDis =  distance(target, MPosP)


%%%% Useful commands for debugging 

% sense = r.ultraScan 
% move = 15;
% r.move(move); 
% turn = pi/2;
% r.turn(turn); 

%%%% Close up at end    
CloseSensor(SENSOR_4); 
COM_CloseNXT(h);
clear all; 