function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = modifyMap1(map); %you need to do this modification yourself
botSim.setMap(map);

%generate some random particles inside the map
num =200; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

% Set scan parameters
numScans = 6;
botSim.generateScanConfig(numScans);
for i=1:num
    particles(i).generateScanConfig(numScans);
end

% Particle weight array (initialise to equal values)
particleWeight = zeros(1,num) + 1/num;
sensorStdDev = 3;
P_zr = zeros(num,1);

newParticles = particles;

Pos = zeros(num,2);
Ang = zeros(num,1);


%% Localisation code
maxNumOfIterations = 100;
n = 0;
count1 = 0;


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
    dampingFactor = 0.00001;
    for i=1:num
        % particle measurement
        measurement = particles(i).ultraScan();
        
        % rotate array and find error
        for j=1:length(botScan)
            shifted = circshift(measurement,j);
            scanError = (mean(abs(botScan-shifted)))^2;
            exponential = exp( (-1*scanError) / (2*sensorStdDev^2) );
            P_zr_temp = exponential/sqrt(2*pi*sensorStdDev^2) - exp(-1*j);
            if P_zr_temp>P_zr(i)
                P_zr(i)=P_zr_temp+dampingFactor;
            end
        end
    end
    
    
    
    
    %% Write code for scoring your particles
    
    % Work out normalisation factor
    normFactor = sum(P_zr);
    
    % Check maximum probability -> if too high then resample all particles
    MaxP = max(P_zr);
    if MaxP < 0.001
        for i = 1:num
            particles(i).randomPose(0);
        end
        particleWeight = zeros(1,num) + 1/num;
    else
        for i=1:num
            particleWeight(i) = P_zr(i)/normFactor;
        end
    end
    
    particleWeight1 = particleWeight;
    
    %% Average position of particles
    for i = 1:num %find bot pos from readings
        Pos(i,:) = particles(i).getBotPos();
        Ang(i,1) = particles(i).getBotAng();
    end
    
    % Mean position based on the particle locations
    MPos = sum(Pos,1)/num; % Mean position of the particles.
    
    PosP(:,1) = Pos(:,1).*particleWeight1';
    PosP(:,2) = Pos(:,2).*particleWeight1';
    
    MPosP = sum(PosP,1)
    
    %calculation of the postion uncertainty
    DivX = (Pos(:,1)-MPosP(1)).^2;
    DivY = (Pos(:,2)-MPosP(2)).^2;
    
    SDx = sqrt((1/num)*sum(DivX));
    SDy = sqrt((1/num)*sum(DivY));
    
    
    %radius of uncertainty based on 2 standard deviations.
    SD = max(SDx,SDy);
    r = 1.5*SD;
    
    % Modified angle dependent on particle weight
    AngP = Ang.*particleWeight1';
    
    MAngP = mod(sum(AngP),2*pi);
    if MAngP > pi
        MAngP=MAngP-2*pi;
    end
    
    DivAng = (Ang(1)-MAngP(1)).^2;
    
    SDAng = sqrt((1/num)*sum(DivAng));
    
    %% Write code for resapling your particles
    % Any particle with weight lower than threshold deleted
    % - New random point based on remaining particles?
    % - Any particles outside walls should be eliminated
    % - Set all particle weights equal
    
    particleCounter = 1;
    for i=1:num
        numSamples = round(particleWeight(i).*num);
        
        if numSamples>0 && particleCounter<=(num-numSamples)
            pos = particles(i).getBotPos();
            ang = particles(i).getBotAng();
            for j=1:numSamples
                newParticles(particleCounter).setBotPos(pos);
                newParticles(particleCounter).setBotAng(ang);
                particleCounter=particleCounter+1;
            end
        end
    end
    
    for i=particleCounter:num
        newParticles(particleCounter).randomPose(0);
    end
    
    % Update particles location for plotting and next iteration
    particles=newParticles;
    
    % reset weights
    particleWeight(1,:) = 1/num;
    P_zr = zeros(num,1);
    
    %% Write code to check for convergence
    
    
    
    %% Write code to decide how to move next
    % Get optimal path
    
    [C I] = min(botScan);
    if C < 5
        move = 2;
        turn = -I*(pi/3);
    else
         disp('planning...')

        optimalPath = Astar( modifiedMap, round(target), round(MPosP));
        
        sOptPath = size(optimalPath);
        if sOptPath(1)>2
            NewPosition = getMovePosition(optimalPath)
        end
        
        % Decide if to move or not based on uncertainty
        disp(strcat('stdDev: ',num2str(SD)));
        if sOptPath(1)>3 && SD<25
            pathAng = atan2(optimalPath(4, 2)-MPosP(2), optimalPath(4, 1)-MPosP(1));
            turn = pathAng - MAngP;
            if SD < 6
                move = distanceLine(MPosP(1), MPosP(2), NewPosition(1), NewPosition(2))
                move = max(move,3);
            else
                move = distance(MPosP, optimalPath(2,:));
            end
            
        elseif SD<8
            converged = 1;
            disp('WOOHOOO, WEVE ARRIVED');
        else
            % Do nothing
            turn=0.5;
            move = 2;
        end
        
        % Check turn is between -pi and pi
        if (turn<-pi)
            turn=turn+2*pi;
        elseif (turn>pi)
            turn=turn-2*pi;
        end
    end
    
    % Set turn and movement noise
    turnNoise = normrnd(0,5,num,1)./(2*pi);
    moveNoise = normrnd(0,0.5,num,1);
    
    botSim.turn(turn); %turn the real robot.
    botSim.move(move); %move the real robot. These movements are recorded for marking
    
    for i =1:num %for all the particles.
        particles(i).turn(turn+turnNoise(i)); %turn the particle in the same way as the real robot
        particles(i).move(move+moveNoise(i)); %move the particle in the same way as the real robot
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
            plot((r*cos(i) + MPosP(1,1)),(r*sin(i) + MPosP(1,2)),'g') %Plots a representation of the uncertanty with a central probability of location
        end
        AngEnd = [MPosP(1)+10*cos(MAngP),MPosP(2)+10*sin(MAngP)];
        plot([MPosP(1),AngEnd(1)],[MPosP(2),AngEnd(2)],'r');
        
        % Plot path+target
        plot(optimalPath(:,1),optimalPath(:,2),'r');
        plot(target(1),target(2),'r*');
        for i = 0:pi/100:2*pi
            plot((5*cos(i) + target(1)),(5*sin(i) + target(2)),'r') %Plots a representation of the uncertanty with a central probability of location
        end
        
        
        drawnow;
    end
    
end
end
