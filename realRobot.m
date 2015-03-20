%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Robotic Systems March 2015 %%%%%%%%%%
%%%%%%%%%% Team LDCA - Lost Robot Cwk %%%%%%%%%%
%%%%%%%%%% Class for real robot task  %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef realRobot < handle
    %%%%%%%%%%%%%
    properties (SetAccess = private, GetAccess = private)
        sense;  % Sensor readings
        s1;     % Ultrasonic sensor
        mA;     % Motor A (Ultrasound motor)
        mB;     % Motor B >>>>>>>>>>>>> I believe motor B is left (but I am trying to work this out from home so forgive if I am wrong)
        mC;     % Motor C >>>>>>>>>>>>> I believe motor C is right (ditto) i.e.  it turns anti-clockwise when given a positive turn angle
        map;    % map coordinates with a copy of the first coordiantes at the end
        mapLines;   % The map stored as a list of lines (for easy line interection)
        inpolygonMapformatX; % The map stored as a polygon for the insidepoly function
        inpolygonMapformatY; % The map stored as a polygon for the insidepoly function
    end
    
    %public properties
    properties
        unmodifiedMap; % map
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %% Function for Sensing
        
        function sense = ultraScan(robot, sense)
            scanNum = 12; %%%%% change to match numScans in localise
            scanAng = 360/scanNum;
            sense = zeros(scanNum,1); % to return sensor measurements in vector form
            s1 = SENSOR_1;
            % scan steps
            for i = 1:scanAng
                sense(i) = GetUltrasonic(s1); % ultrasound value
                mA = NXTMotor('A', 'Power', 80, 'TachoLimit', scanAng);
                mA.SendToNXT();
                mA.WaitFor();
                mA.Stop('brake')
                senseAng(i) = mA.ReadFromNXT();
            end
            
            %print out scan is useful in debug (or use in localise)
            %sense
            
            % Spin back to start position
            mA = NXTMotor('A', 'Power', -100, 'TachoLimit', 360);
            mA.SendToNXT();
            mA.WaitFor();
            mA.Stop('brake');
            senseAng = mA.ReadFromNXT();
            disp(senseAng.Position);
            
            % stops position drift
            if abs(senseAng.Position) > 2
                if senseAng.Position<0
                    power = 20;
                else
                    power = -20;
                end
                mA = NXTMotor('A','Power',power,'TachoLimit',abs(senseAng.Position));
                mA.SendToNXT();
                mA.WaitFor();
                mA.Stop('brake');
            end
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to move robot
        
        function move(robot, move)
            
            % Useful in debug
            % move
            
            % speed is between -100 and 100
            straightSpeed = 80;
            
            % can reverse (reverses when it hits a wall - could change it to reduce number of turns?)
            if move < 0
                straightSpeed = -straightSpeed;
                move = -move;
            end
            
            calibration = 28; % Calibration based on the distance travelled with full 360deg turn of wheels (floor material dependant)
            angle = move*calibration; % (Angle the wheels must turn in order to make move)
            mStraight = NXTMotor([MOTOR_B, MOTOR_C]);
            mStraight.SpeedRegulation   = false;  % not for sync mode
            mStraight.Power             = straightSpeed;
            mStraight.TachoLimit        = round(angle);
            mStraight.ActionAtTachoLimit = 'Brake';
            mStraight.SendToNXT();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to turn Robot
        
        function turn(robot, turn)
            
            % Useful for debug
            % turn
            
            % Turning speed set such that a positive turn angle relates to
            % anti-clockwise turn (I think, double check if going the wrong
            % way)
            turningSpeed = -60;
            calibrationRadii = 6.5; % Roughly based on the wheel radii and calibrated a bit
            calibration = 28; % as before
            
            % Negative turn angle reverses the turn direction
            if turn < 0
                turn = - turn;
                turningSpeed = - turningSpeed;
            end
            
            % arc is the length the wheel needs to move
            arc = calibrationRadii*turn;
            % turn angle is the angle the motor needs to turn to make the
            % arc
            angleTurn = round(arc*calibration);
            
            % Avoid getting stuck on inf
            if turn == 0
                angleTurn = 1;
            end
            
            % set the parameters for the motors
            mTurn1                      = NXTMotor('B');
            mTurn1.Power                = turningSpeed;
            mTurn1.TachoLimit           = angleTurn;
            
            mTurn2                       = NXTMotor('C');
            mTurn2.Power                 = -turningSpeed;
            mTurn2.TachoLimit            = angleTurn;
            
            % send commands 
            mTurn1.SendToNXT();
            mTurn2.SendToNXT();
            mTurn1.WaitFor();
            mTurn2.WaitFor();
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to set the map (taken from botSim)
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to generateScanConfig - can leave blank
        function generateScanConfig(robot, numScans)
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to generateScanConfig - set to debug mode to draw map
        function d = debug(robot)
            d = true;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to drawMap - taken from botSim
        function drawMap(robot)
            plot(robot.map(:,1),robot.map(:,2),'lineWidth',2,'Color','r'); % draws arena
            axis equal; %keeps the x and y scale the same
            hold on; %
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Function to drawBot - taken from botSim
        function drawBot(robot,lineLength,col)
            hold on;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of methods
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
