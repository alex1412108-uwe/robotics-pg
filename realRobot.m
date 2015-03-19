classdef realRobot < handle    
%%%%%%%%%%%%%     
properties (SetAccess = private, GetAccess = private)
    sense;
    SENSOR_1;
    mA = NXTMotor('A');
    mB = NXTMotor('B');
    mC = NXTMotor('C');
    map;    %map coordinates with a copy of the first coordiantes at the end
    mapLines;   %The map stored as a list of lines (for easy line interection)
    inpolygonMapformatX; %The map stored as a polygon for the insidepoly function
    inpolygonMapformatY; %The map stored as a polygon for the insidepoly function
end

%public properties
properties
    unmodifiedMap;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        methods           
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

        function sense = ultraScan(robot, sense)               
            calibrationAng = 15; 
            scanAng = 60;
            sense = zeros(6,1); % to return sensor measurements in vector form 
        % scan in 60 degree steps
        for i = 1:6
            sense(i) = GetUltrasonic(SENSOR_1); % ultrasound value
            mA = NXTMotor('A', 'Power', 50, 'TachoLimit', scanAng);
            mA.SendToNXT();
            mA.WaitFor(); 
            mA.Stop('brake') 
            senseAng(i) = mA.ReadFromNXT();
        end
        % transform sense    ------- I think this should be changed
         sense(1) = sense(1) + calibrationAng; 
         sense(2) = sense(2) + (calibrationAng*cosd(scanAng));
         sense(3) = sense(3) + (calibrationAng*cosd(2*scanAng));
         sense(4) = sense(4) - calibrationAng; 
         sense(5) = sense(5) + (calibrationAng*cosd(2*scanAng));
         sense(6) = sense(6) + (calibrationAng*cosd(scanAng));
        % Spin back
        mA = NXTMotor('A', 'Power', -50, 'TachoLimit', 360);
        mA.SendToNXT();
        mA.WaitFor(); % without this the motor will barely move!
        mA.Stop('brake');
        senseAng = mA.ReadFromNXT();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function move(robot, move) 
        straightSpeed = -50;    
        calibration = 28;
        angle = move*calibration; 
        mStraight = NXTMotor([MOTOR_B, MOTOR_C]);
        mStraight.SpeedRegulation   = false;  % not for sync mode
        mStraight.Power             = straightSpeed; 
        mStraight.TachoLimit        = round(angle);
        mStraight.ActionAtTachoLimit = 'Brake';
        mStraight.SendToNXT();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function turn(robot, turn) 
        turningSpeed = -40; 
        calibrationRadii = 4.9;
        calibration = 28;
        if turn < 0
            turn = - turn; 
            turningSpeed = - turningSpeed;
        end
       
        arc = calibrationRadii*turn;
        angleTurn = round(arc*calibration); 
        
        mTurn1                      = NXTMotor('B'); 
        mTurn1.Power                = turningSpeed;
        mTurn1.TachoLimit           = angleTurn;        
        
        mTurn2                       = NXTMotor('C');   
        mTurn2.Power                 = -turningSpeed;
        mTurn2.TachoLimit            = angleTurn;
        
        mTurn1.SendToNXT();
        mTurn1.WaitFor();     
        mTurn2.SendToNXT();
        mTurn2.WaitFor();
              
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function setMap(robot, map1)
            robot.unmodifiedMap = map1;
            robot.inpolygonMapformatX = cat(1,map1(:,1), map1(1,1));
            robot.inpolygonMapformatY = cat(1,map1(:,2), map1(1,2));
            
            map1(length(map1)+1,:)= map1(1,:);
            robot.map = map1;
            robot.mapLines = zeros(length(robot.map)-1,4);  %each row represents a border of the map
            for i =1:size(robot.mapLines,1)
                robot.mapLines(i,:) = [robot.map(i,:) robot.map(i+1,:)] ;
            end
        end
        
        function generateScanConfig(robot, numScans)
        end
        
        function d = debug(robot)
            d = true; 
        end
        
         function drawMap(robot)
            plot(robot.map(:,1),robot.map(:,2),'lineWidth',2,'Color','r'); % draws arena
            axis equal; %keeps the x and y scale the same
            hold on; %
         end
        
        function drawBot(robot,lineLength,col)
            hold on; 
        end
        
    end
end
