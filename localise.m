 function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

% Set scan parameters
numScans = 10;
botSim.generateScanConfig(numScans);
for i=1:num
    particles(i).generateScanConfig(numScans);
end

% Particle weight array (initialise to equal values)
particleWeight = zeros(1,num) + 1/num;
sensorStdDev = 3;
P_zr = zeros(num,1);

newParticles = particles;

pathPoint = 1;

%% Localisation code
maxNumOfIterations = 10;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
                                    % Returns distance to walls
                                    % NB: can't use crossing points... Duh.

    %% Write code for updating your particles scans
    % Predict probability for particle in current location
    % - P(measurement|location)
    % - - Based on estimated ultrasound error
    dampingFactor = 0.001;
    for i=1:num
        % particle measurement
        measurement = particles(i).ultraScan();
        turnAmount = 1;
        % rotate array and find error
        for j=1:length(botScan)
            shifted = circshift(measurement,j);
            scanError = (mean(abs(botScan-shifted)))^2;
            exponential = exp( (-1*scanError) / (2*sensorStdDev^2) );
            P_zr_temp = exponential/sqrt(2*pi*sensorStdDev^2);
            if P_zr_temp>P_zr(i)
                P_zr(i)=P_zr_temp+dampingFactor;
                turnAmount = j;
            end
%             particles(i).turn((turnAmount-1)*2*pi / length(botScan) - pi/2);
        end
    end
    
    
    
    
    %% Write code for scoring your particles 
    % Score particles based on botScan:particleEstimate
    % Update in particleWeight(i)
    % - Using Bayes Theorem:    (m=measurement, L=location)
    % - - P(L|m) = 
    %         P(m|L) * P(L) /
    %         P(m)
    %
    % - - Where: P(m)=P(m|L)*P(L) + P(m|~L)*P(~L)
    %                  and P(L)+P(~L)=1
    %
    % - NB: If using probabilities as weight, normalise to sum to 1
    normFactor = sum(P_zr);
    
    for i=1:num
        particleWeight(i) = P_zr(i)/normFactor;
    end
    
    
    
    %% Write code for resapling your particles
    % Any particle with weight lower than threshold deleted
    % - New random point based on remaining particles?
    % - Any particles outside walls should be eliminated
    % - Set all particle weights equal
    
    particleCounter = 1;
    for i=1:num
        numSamples = round(particleWeight(i).*300);
        
        if numSamples>0 && particleCounter<=(300-numSamples)
            pos = particles(i).getBotPos();
            ang = particles(i).getBotAng();
            for j=1:numSamples
                newParticles(particleCounter).setBotPos(pos); 
                newParticles(particleCounter).setBotAng(ang); 
                particleCounter=particleCounter+1;
            end
        end
    end
    
    for i=particleCounter:300
        newParticles(particleCounter).randomPose(0);
    end
%     disp(particleCounter);
    
    
    
    
    % Update particles location for plotting and next iteration
    particles=newParticles;
    
    % reset weights
    particleWeight(1,:) = 1/num;
    P_zr = zeros(num,1);
    
    %% Write code to check for convergence 
    % Check how close particles are to each other
    % - if within tolerance, set convergence=1
    
    Pos = zeros(num,2);
    for i = 1:num %find bot pos from readings
        Pos(i,:) = particles(i).getBotPos();
        Ang(i,:) = particles(i).getBotAng();
    end
    
    MPos = sum(Pos,1)/num;
    MAng = sum(Ang,1)/num;
    
    for i = 1:num
        PosX(i,1) = [Pos(i,1)];
        PosY(i,1) = [Pos(i,2)];
%         PosNow{i} = [Pos(i,1),Pos(i,2)];
    end
    for i = 1:num
        DivX(i) = (PosX(i) - MPos(1)).^2;
        DivY(i) = (PosY(i) - MPos(2)).^2;
    end
    
    SDx = sqrt((1/num)*sum(DivX));
    SDy = sqrt((1/num)*sum(DivY));
        
    A = 1;
    for i = 1:num
        dr(i,1) = A * exp(-((DivX(i)/(2*SDx.^2)) + (DivY(i)/(2*SDy.^2))));
    end
       
    r = 2*SDx;
   

    % Number of particles near to actual thing
%     radius = 10;
%     botLocation = botSim.getBotPos();
%     numNearby = 0;
%     for i=1:num
%         particleLocation = particles(i).getBotPos();
%         locationErrorVec = botLocation - particleLocation;
%         locationError = sqrt(sum(locationErrorVec.^2));
%         if locationError<radius
%             numNearby=  numNearby+1;
%         end
%     end
%     disp(numNearby);
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn=0;
    if (n<3)
        %turn = randn(1,1)*2*pi;
        move = 3+3*randn(1,1);
    elseif mod(n,4)==0
        disp('Planning path...');
        optimalPath = Astar(modifiedMap, round(target), round(MPos));
        pathPoint = 1;
    elseif n>4
        if pathPoint>length(optimalPath)
            turn=0;
            move = 3;
        else
            Ang = atan2(optimalPath(pathPoint,2)-MPos(2),optimalPath(pathPoint,1)-MPos(1));
            turn = -1*Ang
            move = distance(MPos,optimalPath(pathPoint,:));
        end
        
        if move>10
            move = 10;
        end

        if pathPoint==length(optimalPath) || distance(MPos,target)<2
            converged=1;
        elseif pathPoint<length(optimalPath)
            pathPoint = pathPoint+1;
        end
    end
    
    
    
    turnNoise = normrnd(0,0.1,num,1)./(2*pi);
    moveNoise = normrnd(0,10,num,1);
   
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn+randn(1,1)/pi);%turnNoise(i)); %turn the particle in the same way as the real robot
        particles(i).move(move+1*randn(1,1));%moveNoise(i)); %move the particle in the same way as the real robot
    end

    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3,'b'); %draw particle with line length 3 and default color
        end
        for i = 0:pi/100:2*pi
            plot((r*cos(i) + MPos(1,1)),(r*sin(i) + MPos(1,2)),'g') %Plots a representation of the uncertanty with a central probability of location
        end
        
        % Plot path
        if n>=4
            disp('plotting path');
            plot(optimalPath(:,1),optimalPath(:,2),'r');
            hold on; 
        end
        
        plot(target(1),target(2),'r*');
        
        drawnow;
    end
end
end
